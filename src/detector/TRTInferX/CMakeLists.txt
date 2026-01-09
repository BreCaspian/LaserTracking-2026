cmake_minimum_required(VERSION 3.18)

project(TRTInferX LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------LICENSE--------------------------------------
set(BL "${CMAKE_BINARY_DIR}/.license_lock")
set(BD "${CMAKE_BINARY_DIR}/_banner")
set(BS "${BD}/launch.sh")
set(BT [==[

                        .-'''-.                                                           .-'''-.     
                       '   _    \                                                        '   _    \   
                     /   /` '.   \                                  .                  /   /` '.   \  
 .-.          .-    .   |     \  '.-.          .-                 .'|                 .   |     \  '  
  \ \        / /    |   '      |  '\ \        / /                <  |                 |   '      |  ' 
   \ \      / /  __ \    \     / /  \ \      / /                  | |                 \    \     / /  
    \ \    / /.:--.'.`.   ` ..' /    \ \    / /_    _  .--------. | | .'''-.     _    _`.   ` ..' /   
     \ \  / // |   \ |  '-...-'`      \ \  / /| '  / | |____    | | |/.'''. \   | '  / |  '-...-'`    
      \ `  / `" __ | |                 \ `  /.' | .' |     /   /  |  /    | |  .' | .' |              
       \  /   .'.''| |                  \  / /  | /  |   .'   /   | |     | |  /  | /  |              
       / /   / /   | |_                 / / |   `'.  |  /    /___ | |     | | |   `'.  |              
   |`-' /    \ \._,\ '/             |`-' /  '   .'|  '/|         || '.    | '.'   .'|  '/             
    '..'      `--'  `"               '..'    `-'  `--' |_________|'---'   '---'`-'  `--'              


Copyright (C) 2026 ROBOMASTER NCST HORIZON — YAO YUZHUO
Licensed under the GNU Affero General Public License v3.0 (AGPL-3.0).
]==])
file(MAKE_DIRECTORY "${BD}")
file(WRITE "${BS}" "set -eu\nl='${BL}'\nif mkdir \"$l\" 2>/dev/null; then\ntrap 'rmdir \"$l\" 2>/dev/null||true' EXIT\nprintf '\\033[1;93m' 1>&2\ncat 1>&2 <<'__B__'\n${BT}\n__B__\nprintf '\\033[0m\\n' 1>&2\nfi\nexec \"$@\"\n")
execute_process(COMMAND /bin/chmod +x "${BS}")
foreach(L C CXX CUDA)
  if(CMAKE_${L}_COMPILER)
    set(CMAKE_${L}_COMPILER_LAUNCHER "${BS}")
  endif()
endforeach()
# ----------------------------------------------------------------------

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
    if(DETECTED_CC MATCHES "^[0-9]+$")
      set(CMAKE_CUDA_ARCHITECTURES "${DETECTED_CC}" CACHE STRING "CUDA arch" FORCE)
      message(STATUS "Auto-detected CMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES}")
    else()
      set(CMAKE_CUDA_ARCHITECTURES 86)
      message(STATUS "NVML unavailable; using default CMAKE_CUDA_ARCHITECTURES=86 (set -DCMAKE_CUDA_ARCHITECTURES to override)")
    endif()
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

add_library(trt_yolo STATIC
  src/Inference.cpp
  src/Api.cpp
  kernel/preprocess.cu
  kernel/postprocess.cu
)

target_include_directories(trt_yolo
  PUBLIC include
)

if(TRT_INCLUDE_DIR)
  target_include_directories(trt_yolo PRIVATE "${TRT_INCLUDE_DIR}")
endif()

if(TRT_LIB_DIR)
  target_link_directories(trt_yolo BEFORE PRIVATE "${TRT_LIB_DIR}")
  set_target_properties(trt_yolo PROPERTIES
    BUILD_RPATH "${TRT_LIB_DIR}"
    INSTALL_RPATH "${TRT_LIB_DIR}"
    BUILD_RPATH_USE_ORIGIN ON
    INSTALL_RPATH_USE_LINK_PATH ON
  )
endif()

set(TRT_LINK_LIBS nvinfer nvinfer_plugin nvonnxparser)
set(TRT_LIB_HINTS "")
if(TRT_LIB_DIR)
  list(APPEND TRT_LIB_HINTS "${TRT_LIB_DIR}")
endif()
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND TRT_LIB_HINTS "$ENV{CONDA_PREFIX}/lib")
endif()
list(APPEND TRT_LIB_HINTS
  "/lib/x86_64-linux-gnu"
  "/usr/lib/x86_64-linux-gnu"
  "/usr/lib"
)

find_library(TRT_NVINFER_LIB NAMES nvinfer PATHS ${TRT_LIB_HINTS} NO_DEFAULT_PATH)
find_library(TRT_NVINFER_PLUGIN_LIB NAMES nvinfer_plugin PATHS ${TRT_LIB_HINTS} NO_DEFAULT_PATH)
find_library(TRT_NVONNXPARSER_LIB NAMES nvonnxparser PATHS ${TRT_LIB_HINTS} NO_DEFAULT_PATH)

if(NOT TRT_NVINFER_LIB OR NOT TRT_NVINFER_PLUGIN_LIB OR NOT TRT_NVONNXPARSER_LIB)
  find_library(TRT_NVINFER_LIB NAMES nvinfer)
  find_library(TRT_NVINFER_PLUGIN_LIB NAMES nvinfer_plugin)
  find_library(TRT_NVONNXPARSER_LIB NAMES nvonnxparser)
endif()

if(TRT_NVINFER_LIB AND TRT_NVINFER_PLUGIN_LIB AND TRT_NVONNXPARSER_LIB)
  set(TRT_LINK_LIBS ${TRT_NVINFER_LIB} ${TRT_NVINFER_PLUGIN_LIB} ${TRT_NVONNXPARSER_LIB})
  get_filename_component(TRT_LIB_DIR_DETECTED "${TRT_NVINFER_LIB}" DIRECTORY)
  message(STATUS "Linking TensorRT from ${TRT_LIB_DIR_DETECTED}")
else()
  message(WARNING "TensorRT libs not found via hints; falling back to linker defaults.")
endif()

target_link_libraries(trt_yolo
  PUBLIC
    ${OpenCV_LIBS}
    CUDA::cudart
    ${TRT_LINK_LIBS}
)

target_compile_options(trt_yolo PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-O3>
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math;-lineinfo;-Xcompiler=-w>
)

add_executable(trt_yolo_example examples/yolo11/main.cpp)
target_link_libraries(trt_yolo_example PRIVATE trt_yolo)
if(TRT_LIB_DIR)
  target_link_directories(trt_yolo_example BEFORE PRIVATE "${TRT_LIB_DIR}")
  set_target_properties(trt_yolo_example PROPERTIES
    BUILD_RPATH "${TRT_LIB_DIR}"
    INSTALL_RPATH "${TRT_LIB_DIR}"
    BUILD_RPATH_USE_ORIGIN ON
    INSTALL_RPATH_USE_LINK_PATH ON
  )
endif()

if(TRT_LIB_DIR_DETECTED)
  set_target_properties(trt_yolo PROPERTIES
    BUILD_RPATH "${TRT_LIB_DIR_DETECTED}"
    INSTALL_RPATH "${TRT_LIB_DIR_DETECTED}"
  )
  set_target_properties(trt_yolo_example PROPERTIES
    BUILD_RPATH "${TRT_LIB_DIR_DETECTED}"
    INSTALL_RPATH "${TRT_LIB_DIR_DETECTED}"
  )
endif()
