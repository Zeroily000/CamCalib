cmake_minimum_required(VERSION 2.8)

project(Calibration)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pedantic -O3 -Wno-long-long -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(OpenCV_DIR /home/andy/opencv/build/)
set(OpenCV_FOUND 1)

#FIND_PACKAGE( OpenCV REQUIRED )
find_package( OpenCV REQUIRED )
# link_directories ( ${OpenCV_LIB_DIR} )
# MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

# include_directories ( ${OpenCV_INCLUDE_DIRS} )
# MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

#INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/)
#include_directories(${CMAKE_SOURCE_DIR}/ ${CMAKE_SOURCE_DIR}/src/)

SET(SRC_FOLDER src)
FILE(GLOB SOURCES "${SRC_FOLDER}/*.cpp")
# 	file(GLOB SOURCES src/*.cpp src/*.h src/*.hpp)


add_executable(calib ${SOURCES})
target_link_libraries(calib ${OpenCV_LIBS})