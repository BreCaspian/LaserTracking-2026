project(detector_registry LANGUAGES CXX)

if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()

add_library(detector_registry STATIC
  src/backend.cpp
)

target_include_directories(detector_registry
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(detector_registry
  PUBLIC
    detector_common
    detector_classical
    ${OpenCV_LIBS}
)

if(DETECTOR_WITH_TRT)
  target_compile_definitions(detector_registry PUBLIC DETECTOR_WITH_TRT)
  target_link_libraries(detector_registry PUBLIC detector_trt_engine)
endif()
