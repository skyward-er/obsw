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

add_executable(parafoil-entry-milano src/entrypoints/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-entry-milano PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-entry-milano PRIVATE MILANO)
sbs_target(parafoil-entry-milano stm32f429zi_skyward_death_stack_x)

add_executable(parafoil-entry-stop_and_go src/entrypoints/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-entry-stop_and_go PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-entry-stop_and_go PRIVATE MOLINELLA STOP_AND_GO)
sbs_target(parafoil-entry-stop_and_go stm32f429zi_skyward_death_stack_x)

add_executable(parafoil-entry-guided src/entrypoints/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-entry-guided PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-entry-guided PRIVATE MOLINELLA AUTOMATIC)
sbs_target(parafoil-entry-guided stm32f429zi_skyward_death_stack_x)

add_executable(parafoil-entry-rotation src/entrypoints/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-entry-rotation PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-entry-rotation PRIVATE MOLINELLA ROTATION)
sbs_target(parafoil-entry-rotation stm32f429zi_skyward_death_stack_x)
#-----------------------------------------------------------------------------#
#                               Parafoil Computer                             #
#-----------------------------------------------------------------------------#

add_executable(parafoil-test-wes src/tests/Parafoil/parafoil-test-WES.cpp ${PARAFOIL_COMPUTER} )
target_include_directories(parafoil-test-wes PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-test-wes PRIVATE PRF_TEST)
sbs_target(parafoil-test-wes stm32f429zi_skyward_death_stack_x)

add_executable(parafoil-test-wing-controller src/tests/Parafoil/parafoil-test-WingController.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-test-wing-controller PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-test-wing-controller PRIVATE PRF_TEST)
sbs_target(parafoil-test-wing-controller stm32f429zi_skyward_death_stack_x)
