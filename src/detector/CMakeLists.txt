cmake_minimum_required(VERSION 3.10)
project(detector LANGUAGES CXX)

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

option(DETECTOR_BUILD_EXAMPLES "Build detector examples" ON)
option(DETECTOR_WITH_HIK_CAMERA "Build hik_camera demo" ON)
option(DETECTOR_WITH_TRT "Build TRTInferX backend" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

add_subdirectory(common)
add_subdirectory(classical)
if(DETECTOR_WITH_TRT)
  add_subdirectory(TRTInferX)
  add_subdirectory(trt_engine)
endif()
add_subdirectory(registry)

if(DETECTOR_BUILD_EXAMPLES)
  add_executable(detector_demo examples/main.cpp)
  target_link_libraries(detector_demo
    PRIVATE
      detector_registry
      detector_common
      ${OpenCV_LIBS}
  )
endif()

if(DETECTOR_BUILD_EXAMPLES AND DETECTOR_WITH_HIK_CAMERA)
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../hik_camera/CMakeLists.txt")
    if(NOT TARGET hik_camera)
      add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../hik_camera ${CMAKE_CURRENT_BINARY_DIR}/hik_camera_build)
    endif()
    add_executable(detector_hik_demo examples/hik_camera_demo.cpp)
    target_link_libraries(detector_hik_demo
      PRIVATE
        detector_registry
        detector_common
        hik_camera
        ${OpenCV_LIBS}
    )
    if(DETECTOR_WITH_TRT)
      target_link_libraries(detector_hik_demo PRIVATE detector_trt_engine)
      target_compile_definitions(detector_hik_demo PRIVATE DETECTOR_WITH_TRT)
    endif()
  else()
    message(WARNING "hik_camera not found; detector_hik_demo will not be built")
  endif()
endif()
