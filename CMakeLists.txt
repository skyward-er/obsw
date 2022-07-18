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
#                                    HIL Tests                                #
#-----------------------------------------------------------------------------#

add_executable(test-HIL src/tests/hardware_in_the_loop/test-HIL/test-HIL.cpp ${HIL_TESTS})
target_include_directories(test-HIL PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(test-HIL PRIVATE HILSimulation)
sbs_target(test-HIL stm32f429zi_skyward_death_stack_v3)

#-----------------------------------------------------------------------------#
#                                Main Computer                                #
#-----------------------------------------------------------------------------#

add_executable(catch-tests-main
    src/tests/Main/catch/catch-tests-main.cpp
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

add_executable(main-test-buzzer src/tests/Main/actuators/test-buzzer.cpp)
sbs_target(main-test-buzzer stm32f429zi_skyward_death_stack_v3)

add_executable(test-radio src/tests/Main/test-radio.cpp ${MAIN_COMPUTER})
target_include_directories(test-radio PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-radio stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-cutter src/tests/Main/actuators/test-cutter.cpp)
sbs_target(main-test-cutter stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-servos src/tests/Main/actuators/test-servos.cpp)
sbs_target(main-test-servos stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-bmx160-calibration src/tests/Main/calibration/test-bmx160-calibration.cpp)
sbs_target(main-test-bmx160-calibration stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-bmx160 src/tests/Main/sensors/test-bmx160.cpp)
target_include_directories(main-test-bmx160 PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-bmx160 stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-mpu9250 src/tests/Main/sensors/test-mpu9250.cpp)
target_include_directories(main-test-mpu9250 PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-mpu9250 stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-ads131 src/tests/Main/sensors/test-ads131.cpp)
sbs_target(main-test-ads131 stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-ms5803 src/tests/Main/sensors/test-ms5803.cpp)
sbs_target(main-test-ms5803 stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-current-sensing src/tests/Main/sensors/test-current-sensing.cpp)
sbs_target(main-test-current-sensing stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-can src/tests/Main/test-can.cpp)
sbs_target(main-test-can stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-leds src/tests/Main/test-leds.cpp)
sbs_target(main-test-leds stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-nas src/tests/Main/test-nas.cpp)
sbs_target(main-test-nas stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-radio src/tests/Main/test-radio.cpp ${MAIN_COMPUTER})
target_include_directories(main-test-radio PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-radio stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-receiver src/tests/Main/test-receiver.cpp)
target_include_directories(main-test-receiver PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-receiver stm32f407vg_stm32f4discovery)

add_executable(main-test-sensors src/tests/Main/test-sensors.cpp ${MAIN_COMPUTER})
target_include_directories(main-test-sensors PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-sensors stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-serial-ports src/tests/Main/test-serial-ports.cpp)
sbs_target(main-test-serial-ports stm32f429zi_skyward_death_stack_v3)

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
#                              Auxiliary Computer                             #
#-----------------------------------------------------------------------------#

add_executable(aux-test-can src/tests/Auxiliary/test-can.cpp)
sbs_target(aux-test-can stm32f429zi_skyward_pyxis_auxiliary)

#-----------------------------------------------------------------------------#
#                                 Wind gallery                                #
#-----------------------------------------------------------------------------#

add_executable(test-flight-tm src/tests/WindGallery/test-flight-tm.cpp)
target_include_directories(test-flight-tm PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-flight-tm stm32f429zi_stm32f4discovery)
