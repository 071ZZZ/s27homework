cmake_minimum_required(VERSION 3.16)
project(s27_auto_aim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCES
    src/preprocessor.cpp
    src/armor_detector.cpp
    src/pnp.cpp
    src/kalman_filter_3d.cpp
    src/coordinate_transformer.cpp
    src/debug_visualizer.cpp
    src/auto_aim_system.cpp
    src/main.cpp
)

add_executable(auto_aim ${SOURCES})

target_link_libraries(auto_aim ${OpenCV_LIBS})
target_link_options(auto_aim PRIVATE -static-libgcc -static-libstdc++)

configure_file(
    ${CMAKE_SOURCE_DIR}/config/calibration.yaml
    ${CMAKE_BINARY_DIR}/config/calibration.yaml
    COPYONLY
)
