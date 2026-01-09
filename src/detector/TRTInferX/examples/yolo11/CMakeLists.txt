cmake_minimum_required(VERSION 3.18)

project(TRTInferExample LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CUDA_ARCHITECTURES)
  execute_process(
    COMMAND nvidia-smi --query-gpu=compute_cap --format=csv,noheader
    OUTPUT_VARIABLE NVIDIA_SMI_CC
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(NVIDIA_SMI_CC)
    string(REPLACE "." "" NVIDIA_SMI_CC "${NVIDIA_SMI_CC}")
    string(REPLACE "\n" ";" NVIDIA_SMI_CC_LIST "${NVIDIA_SMI_CC}")
    list(GET NVIDIA_SMI_CC_LIST 0 DETECTED_CC)
    set(CMAKE_CUDA_ARCHITECTURES "${DETECTED_CC}" CACHE STRING "CUDA arch" FORCE)
    message(STATUS "Auto-detected CMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES}")
  else()
    set(CMAKE_CUDA_ARCHITECTURES 86)
    message(STATUS "Using default CMAKE_CUDA_ARCHITECTURES=86 (set -DCMAKE_CUDA_ARCHITECTURES to override)")
  endif()
endif()

set(TRT_INCLUDE_DIR "" CACHE PATH "TensorRT include directory")
set(TRT_LIB_DIR "" CACHE PATH "TensorRT library directory")
set(CUDA_TOOLKIT_ROOT_DIR "" CACHE PATH "CUDA toolkit root directory")

find_package(OpenCV REQUIRED)

if(CUDA_TOOLKIT_ROOT_DIR)
  list(APPEND CMAKE_PREFIX_PATH "${CUDA_TOOLKIT_ROOT_DIR}")
endif()
find_package(CUDAToolkit REQUIRED)

add_executable(TRTInferExample
  ${PROJECT_SOURCE_DIR}/main.cpp
  ${PROJECT_SOURCE_DIR}/../../src/Inference.cpp
  ${PROJECT_SOURCE_DIR}/../../kernel/preprocess.cu
  ${PROJECT_SOURCE_DIR}/../../kernel/postprocess.cu
)

target_include_directories(TRTInferExample
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../../include
    ${TRT_INCLUDE_DIR}
)

if(TRT_LIB_DIR)
  target_link_directories(TRTInferExample PRIVATE "${TRT_LIB_DIR}")
endif()

target_link_libraries(TRTInferExample
  PRIVATE
    ${OpenCV_LIBS}
    CUDA::cudart
    nvinfer
    nvinfer_plugin
    nvonnxparser
)

target_compile_options(TRTInferExample PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-O3>
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math;-lineinfo>
)
