cmake_minimum_required(VERSION 3.9)
PROJECT (aruco_test)
find_package(OpenCV REQUIRED)
set( NAME_SRC
        aruco_test/charuco_board/detect_board_charuco.cpp)

INCLUDE_DIRECTORIES( ${CMAKCMAKE_BINARY_DIRE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable( aruco_test ${NAME_SRC})
target_link_libraries( aruco_test ${OpenCV_LIBS})



#Above is used for detect_board_charuco.cpp
#Program Arguments for Charuco board: -dp="_path_/detector_params." -w=5 -h=7 -sl=0.04 -ml=0.02 -d=10 -r

# Below is what is used for detect.cpp:

#cmake_minimum_required(VERSION 3.9)
#project(aruco_test)

#set(CMAKE_CXX_STANDARD 11)

#find_package(OpenCV REQUIRED )

#INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
#link_directories( ${CMAKE_BINARY_DIR}/bin)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#add_executable(aruco_test detect.cpp)

#target_link_libraries(aruco_test ${OpenCV_LIBS} )
