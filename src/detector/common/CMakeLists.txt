project(detector_common LANGUAGES CXX)

if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()

if(NOT TARGET common)
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../common/CMakeLists.txt")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../common ${CMAKE_CURRENT_BINARY_DIR}/common_build)
  else()
    message(FATAL_ERROR "common package not found")
  endif()
endif()

add_library(detector_common STATIC
  src/measurement.cpp
)

target_include_directories(detector_common
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(detector_common
  PUBLIC
    common
    ${OpenCV_LIBS}
)
