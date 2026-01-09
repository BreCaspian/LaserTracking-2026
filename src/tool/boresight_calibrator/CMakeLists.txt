cmake_minimum_required(VERSION 3.10)
project(boresight_calibrator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

if(NOT TARGET common)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../common ${CMAKE_CURRENT_BINARY_DIR}/common_build)
endif()
if(NOT TARGET hik_camera)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../hik_camera ${CMAKE_CURRENT_BINARY_DIR}/hik_camera_build)
endif()
if(NOT TARGET detector_registry)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../detector ${CMAKE_CURRENT_BINARY_DIR}/detector_build)
endif()

add_executable(boresight_calibrator
  src/boresight_calibrator.cpp
)

target_include_directories(boresight_calibrator
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(boresight_calibrator
  PRIVATE
    common
    hik_camera
    detector_registry
    detector_common
    ${OpenCV_LIBS}
)

if(TARGET detector_trt_engine)
  target_link_libraries(boresight_calibrator PRIVATE detector_trt_engine)
  target_compile_definitions(boresight_calibrator PRIVATE DETECTOR_WITH_TRT)
endif()
