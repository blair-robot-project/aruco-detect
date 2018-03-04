cmake_minimum_required(VERSION 3.5)
PROJECT (aruco_test)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

find_package(OpenCV)
find_package(Protobuf)
find_package(cppzmq)

set( NAME_SRC
        aruco_test/gen/pose.pb.cc aruco_test/gen/pose.pb.cc
        aruco_test/aruco_marker/detect_single.cpp aruco_test/aruco_marker/detect_single.h)
INCLUDE_DIRECTORIES(${CMAKCMAKE_BINARY_DIRE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)



set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable( aruco_test ${NAME_SRC})

include_directories(${cppzmq_INCLUDE_DIR})
target_link_libraries(aruco_test  ${cppzmq_LIBRARY})
target_link_libraries(aruco_test  ${PROTOBUF_LIBRARIES})
target_link_libraries(aruco_test  ${OpenCV_LIBS})
