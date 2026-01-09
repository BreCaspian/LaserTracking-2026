cmake_minimum_required(VERSION 3.10)
project(detector_trt_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(detector_trt_engine STATIC
  src/trt_detector.cpp
)

target_include_directories(detector_trt_engine
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(detector_trt_engine
  PUBLIC
    detector_common
    trt_yolo
)
