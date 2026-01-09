cmake_minimum_required(VERSION 3.10)
project(gimbal_serial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

add_library(gimbal_serial STATIC
  src/protocol.cpp
  src/serial_port.cpp
)

target_include_directories(gimbal_serial
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

if(NOT TARGET common)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common ${CMAKE_CURRENT_BINARY_DIR}/common_build)
endif()
target_link_libraries(gimbal_serial PUBLIC common)

add_executable(gimbal_serial_demo examples/main.cpp)
target_link_libraries(gimbal_serial_demo PRIVATE gimbal_serial)
