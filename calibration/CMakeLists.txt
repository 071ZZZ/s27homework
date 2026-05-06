cmake_minimum_required(VERSION 3.10)
project(CameraCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(calibrate calibrate.cpp)

target_link_libraries(calibrate ${OpenCV_LIBS})