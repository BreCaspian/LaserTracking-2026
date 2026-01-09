cmake_minimum_required(VERSION 3.10)
project(hik_camera LANGUAGES CXX)

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


Copyright (C) 2026  ROBOMASTER NCST HORIZON  YAO YUZHUO
Licensed under the GNU General Public License v3.0 (GPL-3.0).
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

option(HIK_CAMERA_BUILD_EXAMPLES "Build examples" ON)
option(HIK_CAMERA_ENABLE_CUDA "Enable CUDA features when available" ON)
option(HIK_CAMERA_ENABLE_NPP "Enable NPP demosaic when available" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(HIK_CAMERA_CXX_FLAGS -O3 -Wall -Wextra -Wpedantic)
endif()

# OpenCV
find_package(OpenCV REQUIRED)

if(HIK_CAMERA_ENABLE_CUDA)
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    message(STATUS "CUDA toolkit found; enabling CUDA features")
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
      # CMake 3.22 may not accept "native" for CUDA; pin to Ampere (RTX 3060 = 86).
      set(CMAKE_CUDA_ARCHITECTURES 86)
    endif()
  else()
    message(STATUS "CUDA toolkit not found; CUDA features disabled")
  endif()
endif()

find_path(OPENCV_CUDAIMGPROC_INCLUDE
  NAMES opencv2/cudaimgproc.hpp
  PATHS ${OpenCV_INCLUDE_DIRS}
)
if(OPENCV_CUDAIMGPROC_INCLUDE)
  message(STATUS "OpenCV CUDA modules detected")
endif()

# MVS SDK
set(MVS_ROOT "/opt/MVS" CACHE PATH "MVS SDK root directory")
set(MVS_INCLUDE_DIR "${MVS_ROOT}/include")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(MVS_LIB_DIR "${MVS_ROOT}/lib/aarch64")
else()
  set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")
endif()

if(NOT EXISTS "${MVS_INCLUDE_DIR}")
  message(FATAL_ERROR "MVS include dir not found: ${MVS_INCLUDE_DIR}")
endif()

add_library(hik_camera STATIC
  src/hik_camera.cpp
  src/hik_camera_config.cpp
  src/hik_camera_frame.cpp
  src/hik_camera_gpu.cpp
)

if(HIK_CAMERA_ENABLE_NPP AND CUDAToolkit_FOUND)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  target_sources(hik_camera PRIVATE src/gpu_demosaic_npp.cu)
endif()

target_include_directories(hik_camera
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_directories(hik_camera PUBLIC ${MVS_LIB_DIR})

target_link_libraries(hik_camera
  PUBLIC
    ${OpenCV_LIBS}
    MvCameraControl
    MvUsb3vTL
    MVGigEVisionSDK
    MediaProcess
    FormatConversion
    MVRender
)

if(HIK_CAMERA_CXX_FLAGS)
  target_compile_options(hik_camera PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${HIK_CAMERA_CXX_FLAGS}>)
endif()

if(HIK_CAMERA_ENABLE_CUDA AND CUDAToolkit_FOUND)
  target_compile_options(hik_camera PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-w>
  )
endif()

if(HIK_CAMERA_ENABLE_CUDA AND CUDAToolkit_FOUND)
  target_compile_definitions(hik_camera PUBLIC HIK_CAMERA_WITH_CUDA)
  target_include_directories(hik_camera PUBLIC ${CUDAToolkit_INCLUDE_DIRS})
  target_link_libraries(hik_camera PUBLIC CUDA::cudart)
endif()

if(OPENCV_CUDAIMGPROC_INCLUDE)
  target_compile_definitions(hik_camera PUBLIC HIK_CAMERA_WITH_OPENCV_CUDA)
endif()

if(HIK_CAMERA_ENABLE_NPP AND CUDAToolkit_FOUND)
  target_compile_definitions(hik_camera PUBLIC HIK_CAMERA_WITH_NPP)
  target_link_libraries(hik_camera PUBLIC CUDA::nppc CUDA::nppicc)
endif()

if(HIK_CAMERA_BUILD_EXAMPLES)
  add_executable(hik_camera_demo examples/main.cpp)
  target_link_libraries(hik_camera_demo PRIVATE hik_camera)
endif()
