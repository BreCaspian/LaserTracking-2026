project(detector_classical LANGUAGES CXX)

if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()

add_library(detector_classical STATIC
  src/pingpong_detector.cpp
)

target_include_directories(detector_classical
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(detector_classical
  PUBLIC
    detector_common
    ${OpenCV_LIBS}
)
