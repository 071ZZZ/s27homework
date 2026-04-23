cmake_minimum_required(VERSION 3.10)
project(ArmorDetector)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)

add_executable(armor_detector armor_detector.cpp)
target_link_libraries(armor_detector ${OpenCV_LIBS})