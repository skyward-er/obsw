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

project(SkywardParafoilTest)

# -----------------------------------------------------------------------------#
#                                Parafoil mockup                               #
# -----------------------------------------------------------------------------#

add_executable(parafoil-mockup-entry src/entrypoints/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-mockup-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(parafoil-mockup-entry stm32f429zi_skyward_parafoil)

# add_executable(parafoil-test-bme280 src/tests/Parafoil/parafoil-test-bme280.cpp)
# sbs_target(parafoil-test-bme280 stm32f429zi_skyward_parafoil)

# add_executable(parafoil-test-ublox-uart src/tests/Parafoil/parafoil-test-ublox-uart.cpp)
# sbs_target(parafoil-test-ublox-uart stm32f429zi_skyward_parafoil)
