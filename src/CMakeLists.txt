cmake_minimum_required(VERSION 3.10)
project(LaserTracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

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

option(BUILD_CONTROL "Build control module" ON)
option(BUILD_DETECTOR "Build detector module" ON)
option(BUILD_HIK_CAMERA "Build hik_camera module" ON)
option(BUILD_GIMBAL_SERIAL "Build gimbal_serial module" ON)
option(BUILD_TOOLS "Build tools" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

add_subdirectory(common)

if(BUILD_GIMBAL_SERIAL)
  add_subdirectory(gimbal_serial)
endif()

if(BUILD_HIK_CAMERA)
  add_subdirectory(hik_camera)
endif()

if(BUILD_DETECTOR)
  add_subdirectory(detector)
endif()

if(BUILD_CONTROL)
  add_subdirectory(control)
endif()

if(BUILD_TOOLS)
  add_subdirectory(tool/parallax_estimator)
  add_subdirectory(tool/boresight_calibrator)
endif()
