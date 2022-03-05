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
include(skyward-boardcore/cmake/sbs.cmake)

#-----------------------------------------------------------------------------#
#                                   Project                                   #
#-----------------------------------------------------------------------------#

project(SkywardParafoilTest)

#-----------------------------------------------------------------------------#
#                                Main Computer                                #
#-----------------------------------------------------------------------------#

add_executable(catch-tests-main
    src/tests/Main/catch/catch-tests-entry.cpp
    src/tests/Main/catch/test-airbrakes.cpp
    src/tests/Main/catch/test-deployment.cpp
    ${MAIN_COMPUTER}
)
target_include_directories(catch-tests-main PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(catch-tests-main PRIVATE USE_MOCK_PERIPHERALS)
sbs_target(catch-tests-main stm32f429zi_skyward_death_stack_x)
sbs_catch_test(catch-tests-main)

#-----------------------------------------------------------------------------#
#                              Parafoil Computer                              #
#-----------------------------------------------------------------------------#

add_executable(parafoil-entry
	src/entrypoints/Parafoil/parafoil-entry.cpp
	${PARAFOIL_COMPUTER}
)
target_include_directories(parafoil-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(parafoil-entry stm32f429zi_stm32f4discovery)

add_executable(parafoil-test-bme280 src/tests/Parafoil/parafoil-test-bme280.cpp)
sbs_target(parafoil-test-bme280 stm32f429zi_stm32f4discovery)

add_executable(parafoil-test-ublox-uart src/tests/Parafoil/parafoil-test-ublox-uart.cpp)
sbs_target(parafoil-test-ublox-uart stm32f429zi_stm32f4discovery)
