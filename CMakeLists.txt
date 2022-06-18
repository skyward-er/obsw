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

#-----------------------------------------------------------------------------#
#                                   Project                                   #
#-----------------------------------------------------------------------------#

project(SkywardParafoilTest)

#-----------------------------------------------------------------------------#
#                              Flight entrypoints                             #
#-----------------------------------------------------------------------------#

add_executable(main-entry src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-entry stm32f429zi_skyward_death_stack_v3)

#-----------------------------------------------------------------------------#
#                                Main Computer                                #
#-----------------------------------------------------------------------------#

add_executable(catch-tests-main
    src/tests/Main/catch/catch-tests-entry.cpp
    src/tests/Main/catch/test-airbrakes.cpp
    src/tests/Main/catch/test-ada.cpp
    src/tests/Main/catch/test-deployment.cpp
    src/tests/Main/catch/test-fsr.cpp
    src/tests/Main/catch/test-nas.cpp
    ${MAIN_COMPUTER_CATCH}
)
target_include_directories(catch-tests-main PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(catch-tests-main PRIVATE USE_MOCK_PERIPHERALS)
sbs_target(catch-tests-main stm32f429zi_skyward_death_stack_v3)
sbs_catch_test(catch-tests-main)

add_executable(death-stack-v3-testsuite src/tests/Main/death-stack-v3-testsuite.cpp)
target_include_directories(death-stack-v3-testsuite PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(death-stack-v3-testsuite stm32f429zi_skyward_death_stack_v3)

add_executable(catch-tests-entry src/tests/Main/catch/catch-tests-entry.cpp ${MAIN_COMPUTER})
target_include_directories(catch-tests-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(catch-tests-entry stm32f429zi_skyward_death_stack_v3)

add_executable(test-radio src/tests/Main/test-radio.cpp ${MAIN_COMPUTER})
target_include_directories(test-radio PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-radio stm32f429zi_skyward_death_stack_v3)

add_executable(test-sensors src/tests/Main/test-sensors.cpp ${MAIN_COMPUTER})
target_include_directories(test-sensors PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-sensors stm32f429zi_skyward_death_stack_v3)

add_executable(test-receiver src/tests/Main/test-receiver.cpp)
target_include_directories(test-receiver PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-receiver stm32f407vg_stm32f4discovery)

add_executable(test-Main-bmx160 src/tests/Main/sensors/test-Main-bmx160.cpp)
target_include_directories(test-Main-bmx160 PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-Main-bmx160 stm32f429zi_skyward_death_stack_v3)

add_executable(test-Main-mpu9250 src/tests/Main/sensors/test-Main-mpu9250.cpp)
target_include_directories(test-Main-mpu9250 PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-Main-mpu9250 stm32f429zi_skyward_death_stack_v3)

add_executable(test-Main-ads131 src/tests/Main/sensors/test-Main-ads131.cpp)
sbs_target(test-Main-ads131 stm32f429zi_skyward_death_stack_v3)

add_executable(test-Main-ms5803 src/tests/Main/sensors/test-Main-ms5803.cpp)
sbs_target(test-Main-ms5803 stm32f429zi_skyward_death_stack_v3)

add_executable(test-Main-buzzer src/tests/Main/actuators/test-Main-buzzer.cpp)
sbs_target(test-Main-buzzer stm32f429zi_skyward_death_stack_v3)

add_executable(test-bmx160-calibration src/tests/Main/calibration/test-bmx160-calibration.cpp)
sbs_target(test-bmx160-calibration stm32f429zi_skyward_death_stack_v3)

add_executable(test-Main-nas src/tests/Main/test-nas.cpp)
sbs_target(test-Main-nas stm32f429zi_skyward_death_stack_v3)

#-----------------------------------------------------------------------------#
#                              Parafoil Computer                              #
#-----------------------------------------------------------------------------#

# add_executable(parafoil-entry src/entrypoints/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
# target_include_directories(parafoil-entry PRIVATE ${OBSW_INCLUDE_DIRS})
# sbs_target(parafoil-entry stm32f429zi_stm32f4discovery)

# add_executable(parafoil-test-bme280 src/tests/Parafoil/parafoil-test-bme280.cpp)
# sbs_target(parafoil-test-bme280 stm32f429zi_stm32f4discovery)

# add_executable(parafoil-test-ublox-uart src/tests/Parafoil/parafoil-test-ublox-uart.cpp)
# sbs_target(parafoil-test-ublox-uart stm32f429zi_stm32f4discovery)

#-----------------------------------------------------------------------------#
#                                 Wind gallery                                #
#-----------------------------------------------------------------------------#

add_executable(test-flight-tm src/tests/WindGallery/test-flight-tm.cpp)
target_include_directories(test-flight-tm PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-flight-tm stm32f429zi_stm32f4discovery)
