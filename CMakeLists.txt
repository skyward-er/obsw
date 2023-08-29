# Copyright (c) 2021 Skyward Experimental Rocketry
# Author: Damiano Amatruda
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

cmake_minimum_required(VERSION 3.16)
enable_testing()
include(skyward-boardcore/cmake/sbs.cmake)

# -----------------------------------------------------------------------------#
#                                    Project                                   #
# -----------------------------------------------------------------------------#

project(OnBoardSoftware)

#-----------------------------------------------------------------------------#
#                              Flight entrypoints                             #
#-----------------------------------------------------------------------------#

add_executable(test-hil src/entrypoints/HIL/test-hil.cpp ${HIL})
target_include_directories(test-hil PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(test-hil PRIVATE HILTest)
sbs_target(test-hil stm32f767zi_compute_unit)

add_executable(main-entry src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-entry stm32f767zi_skyward_death_stack_v4)

add_executable(main-entry-hil src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${MAIN_HIL})
target_include_directories(main-entry-hil PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-hil PRIVATE HILMain EUROC)
sbs_target(main-entry-hil stm32f767zi_skyward_death_stack_v4)

add_executable(main-entry-roccaraso src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-roccaraso PRIVATE ROCCARASO)
sbs_target(main-entry-roccaraso stm32f767zi_skyward_death_stack_v4)

add_executable(main-entry-roccaraso-hil src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${MAIN_HIL})
target_include_directories(main-entry-roccaraso-hil PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-roccaraso-hil PRIVATE ROCCARASO HILMain)
sbs_target(main-entry-roccaraso-hil stm32f767zi_skyward_death_stack_v4)

add_executable(main-entry-euroc src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-euroc PRIVATE EUROC)
sbs_target(main-entry-euroc stm32f767zi_skyward_death_stack_v4)

add_executable(main-entry-euroc-hil src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${MAIN_HIL})
target_include_directories(main-entry-euroc-hil PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-euroc-hil PRIVATE EUROC HILMain)
sbs_target(main-entry-euroc-hil stm32f767zi_skyward_death_stack_v4)

add_executable(payload-entry-roccaraso src/entrypoints/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-roccaraso PRIVATE ROCCARASO)
sbs_target(payload-entry-roccaraso stm32f767zi_skyward_death_stack_v4)

add_executable(payload-entry-euroc src/entrypoints/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-euroc PRIVATE EUROC)
sbs_target(payload-entry-euroc stm32f767zi_skyward_death_stack_v4)

add_executable(motor-entry src/entrypoints/Motor/motor-entry.cpp ${MOTOR_SOURCES})
target_include_directories(motor-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(motor-entry stm32f767zi_gemini_motor)

add_executable(rig-entry src/entrypoints/RIG/rig-entry.cpp ${RIG_COMPUTER})
target_include_directories(rig-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(rig-entry stm32f429zi_skyward_rig)

add_executable(con_rig-entry src/entrypoints/con_RIG/con_rig-entry.cpp ${CON_RIG_COMPUTER})
target_include_directories(con_rig-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(con_rig-entry stm32f429zi_stm32f4discovery)
