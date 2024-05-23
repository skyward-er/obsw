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

add_executable(base-groundstation-entry
    src/entrypoints/Groundstation/base-groundstation-entry.cpp
    ${GROUNDSTATION_COMMON} ${GROUNDSTATION_BASE}
)
target_include_directories(base-groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(base-groundstation-entry stm32f767zi_gemini_gs)

add_executable(nokia-groundstation-entry
    src/entrypoints/Groundstation/nokia-groundstation-entry.cpp
    ${GROUNDSTATION_COMMON} ${GROUNDSTATION_NOKIA}
)
target_include_directories(nokia-groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(nokia-groundstation-entry stm32f429zi_skyward_groundstation_v2)

add_executable(automated-antennas-entry
    src/entrypoints/Groundstation/Automated/automated-antennas-entry.cpp
    ${ANTENNAS} ${GROUNDSTATION_COMMON} ${GROUNDSTATION_AUTOMATED}
)
target_include_directories(automated-antennas-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(automated-antennas-entry stm32f767zi_nucleo)

add_executable(test-automated-radio
    src/entrypoints/Groundstation/Automated/test-automated-radio.cpp
    ${ANTENNAS} ${GROUNDSTATION_COMMON} ${GROUNDSTATION_AUTOMATED}
)
target_include_directories(test-automated-radio PRIVATE ${OBSW_INCLUDE_DIRS})
# target_compile_definitions(test-automated-radio PRIVATE NO_SD_LOGGING)
sbs_target(test-automated-radio stm32f767zi_automated_antennas)

add_executable(test-steps src/entrypoints/Groundstation/Automated/test-steps.cpp ${ANTENNAS})
target_include_directories(test-steps PRIVATE ${OBSW_INCLUDE_DIRS})
# target_compile_definitions(test-steps PRIVATE NO_SD_LOGGING)
sbs_target(test-steps stm32f767zi_automated_antennas)

add_executable(test-actuators src/entrypoints/Groundstation/Automated/test-actuators.cpp
    ${ANTENNAS} ${GROUNDSTATION_COMMON} ${GROUNDSTATION_AUTOMATED}
)
target_include_directories(test-actuators PRIVATE ${OBSW_INCLUDE_DIRS})
# target_compile_definitions(test-actuators PRIVATE NO_SD_LOGGING)
sbs_target(test-actuators stm32f767zi_automated_antennas)

# add_executable(test-smcontroller src/entrypoints/Groundstation/Automated/test-smcontroller.cpp ${GROUNDSTATION_COMMON})
# target_include_directories(test-smcontroller PRIVATE ${OBSW_INCLUDE_DIRS})
# sbs_target(test-smcontroller stm32f767zi_nucleo)
