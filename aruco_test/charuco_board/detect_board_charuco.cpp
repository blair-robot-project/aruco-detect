#include <opencv2/highgui.hpp>
#include "opencv2/aruco/charuco.hpp"
#include "../gen/pose.pb.h"

#include <iostream>
#include <opencv/cv.hpp>
#include <zmq.hpp>

using namespace std;
using namespace cv;
using namespace proto;


namespace {
	const char *about = "Pose estimation using a ChArUco board";
	const char *keys =
			"{w        |       | Number of squares in X direction }"
					"{h        |       | Number of squares in Y direction }"
					"{sl       |       | Square side length (in meters) }"
					"{ml       |       | Marker side length (in meters) }"
					"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
					"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
					"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
					"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
					"{c        |       | Output file with calibrated camera parameters }"
					"{v        |       | Input from video file, if ommited, input comes from camera }"
					"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
					"{dp       |       | File of marker detector parameters }"
					"{rs       |       | Apply refind strategy }"
					"{r        |       | show rejected candidates too }";
}
/**
 * -w=5 -h=7 -sl=.033 -ml=.025 -d=11 -dp="/home/paragon/CLionProjects/aruco-detect/aruco_test/charuco_board/detector_params.yml" -c="/home/paragon/CLionProjects/aruco-detect/aruco_test/charuco_board/default.yml"
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
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


template<typename T>
string str(T begin, T end)
{
	stringstream ss;
	bool first = true;
	for (; begin != end; begin++)
	{
		if (!first)
			ss << ", ";
		ss << *begin;
		first = false;
	}
	return ss.str();
}

/**
 */
int main(int argc, const char *const argv[]){
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	if (argc < 6) {
		parser.printMessage();
		return 0;
	}
	int squaresX = parser.get<int>("w");
	int squaresY = parser.get<int>("h");
	float squareLength = parser.get<float>("sl");
	float markerLength = parser.get<float>("ml");
	int dictionaryId = parser.get<int>("d");
	bool showRejected = parser.has("r");
	bool refindStrategy = parser.has("rs");
	int camId = parser.get<int>("ci");

	String video;
	if (parser.has("v")) {
		video = parser.get<String>("v");
	}

	Mat camMatrix, distCoeffs;
	if (parser.has("c")) {
		cout<< "reading camera parameters" << endl;
		cout << parser.get<string>("c") << endl;
		bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
		if (!readOk) {
			cerr << "Invalid camera file" << endl;
			return 0;
		}
		cout<< camMatrix.total()  << endl;
	}

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	if (parser.has("dp")) {
		bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
		if (!readOk) {
			cerr << "Invalid detector parameters file" << endl;
			return 0;
		}
	}



	if (!parser.check()) {
		parser.printErrors();
		return 0;
	}


	Ptr<aruco::Dictionary> dictionary =
			aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

	VideoCapture inputVideo;
	int waitTime;
	if (!video.empty()) {
		inputVideo.open(video);
		waitTime = 0;
	} else {
		inputVideo.open(camId);
		waitTime = 10;
	}

	GOOGLE_PROTOBUF_VERIFY_VERSION;


	std::string msg_str;

	//  Prepare our context and socket
	zmq::context_t context(1);
	zmq::socket_t socket(context, ZMQ_PAIR);
	std::cout << "Connecting to server" << std::endl;
	socket.connect("tcp://0.0.0.0:5000");



	float axisLength = 0.5f * ((float) min(squaresX, squaresY) * (squareLength));

	// create charuco board object
	Ptr<aruco::CharucoBoard> charucoboard =
			aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();


	double totalTime = 0;
	int totalIterations = 0;
	CameraPose pose;

	while (inputVideo.grab()) {
		Mat image, imageCopy;
		inputVideo.retrieve(image);

		double tick = (double) getTickCount();

		vector<int> markerIds, charucoIds;
		vector<vector<Point2f> > markerCorners, rejectedMarkers;
		vector<Point2f> charucoCorners;
		Vec3d rvec, tvec;

		// detect markers
		aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
		                     rejectedMarkers);

		// refind strategy to detect more markers
		if (refindStrategy)
			aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
			                             camMatrix, distCoeffs);

		// interpolate charuco corners
		int interpolatedCorners = 0;
		if (markerIds.size() > 0)
			interpolatedCorners =
					aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
					                                 charucoCorners, charucoIds, camMatrix, distCoeffs);

		// estimate charuco board pose
		bool validPose = false;
		if (camMatrix.total() != 0){
			//tvec translation vector, rvec rotation vector
			validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
			                                            camMatrix, distCoeffs, rvec, tvec);
			if(validPose){
				pose.set_x(tvec[0]);
				pose.set_y(tvec[1]);
				pose.set_z(tvec[2]);
			}
		}

		double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
		totalTime += currentTime;
		totalIterations++;
		if (totalIterations % 30 == 0) {
			cout << "Detection Time = " << currentTime * 1000 << " ms "
			     << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
		}

		// draw results
		image.copyTo(imageCopy);
		if (markerIds.size() > 0) {
			aruco::drawDetectedMarkers(imageCopy, markerCorners);
		}

		if (showRejected && rejectedMarkers.size() > 0)
			aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

		if (interpolatedCorners > 0) {
			Scalar color;
			color = Scalar(255, 0, 0);
			aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
		}

		if (validPose) {
			aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);


			msg_str = pose.SerializeAsString();

			zmq::message_t sendRequest(msg_str.size());

            //copy serialized pose into message
			memcpy((void*) sendRequest.data(), msg_str.c_str(), msg_str.size());

			socket.send(sendRequest);
		}
		imshow("out", imageCopy);


		char key = (char) waitKey(waitTime);
		if (key == 27) break;

	}

	google::protobuf::ShutdownProtobufLibrary();
}