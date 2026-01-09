cmake_minimum_required(VERSION 3.10)
project(parallax_estimator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

add_library(parallax_estimator STATIC
  src/parallax_estimator.cpp
)

target_include_directories(parallax_estimator
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(parallax_estimator
  PUBLIC
    ${OpenCV_LIBS}
)

add_executable(parallax_demo examples/parallax_demo.cpp)
target_link_libraries(parallax_demo PRIVATE parallax_estimator)
