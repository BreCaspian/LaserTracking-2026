cmake_minimum_required(VERSION 3.10)
project(control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CONTROL_BUILD_EXAMPLES "Build control examples" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

if(NOT TARGET common)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common_build)
endif()

add_library(control STATIC
  src/controller.cpp
  src/config.cpp
)

target_include_directories(control
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(control
  PUBLIC
    common
    ${OpenCV_LIBS}
)

if(CONTROL_BUILD_EXAMPLES)
  add_executable(control_demo examples/control_demo.cpp)
  target_link_libraries(control_demo PRIVATE control common)
  if(NOT TARGET hik_camera)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../hik_camera ${CMAKE_CURRENT_BINARY_DIR}/hik_camera_build)
  endif()
  if(NOT TARGET detector_registry)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../detector ${CMAKE_CURRENT_BINARY_DIR}/detector_build)
  endif()
  if(NOT TARGET gimbal_serial)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../gimbal_serial ${CMAKE_CURRENT_BINARY_DIR}/gimbal_serial_build)
  endif()
  add_executable(control_system_demo examples/system_demo.cpp)
  target_link_libraries(control_system_demo PRIVATE control detector_registry detector_common hik_camera gimbal_serial common)
  if(TARGET detector_trt_engine)
    target_link_libraries(control_system_demo PRIVATE detector_trt_engine)
    target_compile_definitions(control_system_demo PRIVATE DETECTOR_WITH_TRT)
  endif()
endif()
