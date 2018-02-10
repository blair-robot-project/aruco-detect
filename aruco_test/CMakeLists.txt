cmake_minimum_required(VERSION 3.9)
project(aruco_test)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED )

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#add_executable(aruco_test main.cpp)
add_executable(aruco_test detect.cpp)

target_link_libraries(aruco_test ${OpenCV_LIBS} )