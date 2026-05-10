cmake_minimum_required(VERSION 3.10)

project(kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(homework_2
    homework_2.cpp
)

target_link_libraries(homework_2
    ${OpenCV_LIBS}
)

target_include_directories(homework_2 PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)