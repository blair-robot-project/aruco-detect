#include <iostream>
#include <zmq.hpp>
#include <string>
#include <google/protobuf/stubs/common.h>
#include <opencv/cv.hpp>
#include "gen/pose.pb.h"

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


int main() {

    GOOGLE_PROTOBUF_VERIFY_VERSION;


    //set up zmq

    zmq::context_t context(1);


    zmq::socket_t socket(context, ZMQ_PAIR);

    socket.bind("tcp://*:5000");

    proto::CameraPose pose;

    std::cout << "Starting loop" << std::endl;
    while(true){

        std::cout<<"trying to recieve" << std::endl;

        zmq::message_t recieved;
        socket.recv(&recieved);



        std::cout << "[" << currentDateTime() <<"] recieved " <<
                  std::string(static_cast<char*>(recieved.data()), recieved.size()) << std::endl;

        char key = (char) cv::waitKey(10);
        if (key == 27) break;
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;

}