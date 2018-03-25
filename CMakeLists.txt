cmake_minimum_required(VERSION 3.5)
PROJECT (aruco_test)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

find_package(OpenCV)
find_package(Protobuf)
find_package(cppzmq)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")



#SET(CMAKE_SYSTEM_NAME Windows)

set( NAME_SRC
        aruco_test/gen/pose.pb.cc aruco_test/gen/pose.pb.cc
        aruco_test/aruco_marker/detect_single.cpp aruco_test/aruco_marker/detect_single.h)
INCLUDE_DIRECTORIES("/usr/local/lib")
link_directories( ${CMAKE_BINARY_DIR}/bin)

set(GCC_CXX_FLAGS ${GCC_CXX_FLAGS} ${CMAKE_EXE_LINKER_FLAGS})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable( aruco_test ${NAME_SRC})

set(cppzmq_INCLUDE_DIR "/usr/local/lib")

set(cppzmq_LIBRARY "/usr/local/lib/libzmq.a")
set(PROTOBUF_LIBRARIES "/usr/local/lib/libprotobuf.a")

target_link_libraries(aruco_test ${cppzmq_LIBRARY} ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS})
message(${OpenCV_LIBS})
