#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>

#include <iostream>
#include <zmq.hpp>
#include <google/protobuf/stubs/common.h>

using namespace std;
using namespace cv;
//using namespace proto;

namespace {
    const char* about = "Pose estimation using a ArUco Planar Grid board";
    const char* keys  =
            "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
                    "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
                    "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
                    "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
                    "{v        |       | Input from video file, if ommited, input comes from camera }"
                    "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
                    "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
                    "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
                    "{dp       |       | File of marker detector parameters }"
                    "{p        |       | full ip to send packetes to ex. \"tcp://0.0.0.0:5000\"}";
}

/**
 * example args
 * -ci=1 -l=.195 -d=11 -dp="/home/paragon/CLionProjects/aruco-detect/aruco_test/charuco_board/detector_params.yml" -c="/home/paragon/CLionProjects/aruco-detect/cameraParameters.yml"
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    string s;
    fs["cameraResolution"] >> s;
    cout << s << endl;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}


/**
 * Convert opencv angles to Yaw Pictch and Roll
 *
 * @param R Input matrix, containing rotation around x y and z in order
 * @return the input matrix converted to Euler angles
 */
Vec3d rotationMatrixToEulerAngles(Mat &R) {
    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);

}

/**
 */
int main(int argc, const char *const argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");


    int camId = parser.get<int>("ci");

    Mat camMatrix, distCoeffs;

    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
    int video;
    if(parser.has("v")) {
        video = parser.get<int>("v");
    }


    String port;
    if(parser.has("p")) {
        port = parser.get<String>("p");
    } else {
        cerr << "No ip given" << endl;
        return 0;
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    //Pose object {x y z pitch roll yaw}
    //CameraPose pose;

    //Angles calculated, {x y z}
    Mat rotationAngles;

    //Angles we send, {pitch, roll, yaw}
    Vec3d taitBryanAngles;

    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    //Open a video input, if no user input exists, use default camera
    VideoCapture inputVideo;
    try {
        inputVideo.open(video);
    }catch(const exception& e){
        inputVideo.open(0);
    }
    int waitTime=10;

  //  GOOGLE_PROTOBUF_VERIFY_VERSION;
    std::string msg_str;

    //  Prepare our context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PAIR);
    std::cout << "Connecting to server" << std::endl;

    socket.connect(port);

    float axisLength = 0.5f * markerLength;


    double totalTime = 0;

    int totalIterations = 0;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector<Vec3d> rvecs, tvecs;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if(ids.size() > 0)
                    aruco::estimatePoseSingleMarkers(corners, markerLength,camMatrix, distCoeffs, rvecs, tvecs);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        for(int i = 0; i < ids.size(); i++) {
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], axisLength);
        }

        if(totalIterations % 30 == 0){
            for(int i = 0; i < ids.size(); i++) {
                cout << "Position vectors: " << tvecs[i][0] << " " << tvecs[i][1] << " " << tvecs[i][2] <<endl;

//                pose.set_x(tvecs[i][0]);
//                pose.set_y(tvecs[i][1]);
//                pose.set_z(tvecs[i][2]);


                rotationAngles = Mat(rvecs[i], true);

                taitBryanAngles = rotationMatrixToEulerAngles(rotationAngles);

//                cout << "Rotation vectors: " << taitBryanAngles[0] << " " << taitBryanAngles[1] << " " << taitBryanAngles[2] << endl;

//                pose.set_yaw(taitBryanAngles[0]);
//                pose.set_pitch(taitBryanAngles[1]);
//                pose.set_roll(taitBryanAngles[2]);


//                msg_str = pose.SerializeAsString();

                zmq::message_t request(msg_str.size());

                memcpy((void*) request.data(), msg_str.c_str(), msg_str.size());


                socket.send(request);

            }

        }

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

    //Generate board
//        Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(3, 6, .15, .13,
//                                                                     aruco::getPredefinedDictionary(11));
//
//        cv::Mat outputImage;
//        board->draw(cv::Size(800, 800), outputImage, 10, 1);
//        imshow("board",outputImage);
//         imwrite("/home/paragon/charucoBoards/ProposedBoard20x40.jpg", outputImage);
    return 0;
}


