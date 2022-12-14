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

add_executable(auxiliary-entry src/entrypoints/Auxiliary/auxiliary-entry.cpp ${AUXILIARY_COMPUTER})
target_include_directories(auxiliary-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(auxiliary-entry stm32f429zi_skyward_pyxis_auxiliary)

add_executable(main-entry-euroc src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-euroc PRIVATE EUROC INTERP)
sbs_target(main-entry-euroc stm32f429zi_skyward_death_stack_v3)

add_executable(main-entry-roccaraso src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-roccaraso PRIVATE ROCCARASO)
sbs_target(main-entry-roccaraso stm32f429zi_skyward_death_stack_v3)

add_executable(main-entry-milano src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-milano PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-milano PRIVATE MILANO BUZZER_LOW INTERP USE_SERIAL_TRANSCEIVER)
sbs_target(main-entry-milano stm32f429zi_skyward_death_stack_v3)

add_executable(main-entry-hil-euroc src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${HIL})
target_include_directories(main-entry-hil-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-hil-euroc PRIVATE EUROC BUZZER_LOW INTERP HILSimulation)
sbs_target(main-entry-hil-euroc stm32f429zi_skyward_death_stack_v3)

add_executable(main-entry-hil-roccaraso src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${HIL})
target_include_directories(main-entry-hil-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-hil-roccaraso PRIVATE ROCCARASO BUZZER_LOW INTERP HILSimulation)
sbs_target(main-entry-hil-roccaraso stm32f429zi_skyward_death_stack_v3)

add_executable(main-entry-hil-milano src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${HIL})
target_include_directories(main-entry-hil-milano PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-hil-milano PRIVATE MILANO BUZZER_LOW INTERP HILSimulation)
sbs_target(main-entry-hil-milano stm32f429zi_skyward_death_stack_v3)

add_executable(main-entry-hil-maker-faire src/entrypoints/Main/main-entry-maker-faire.cpp ${MAIN_COMPUTER} ${HIL})
# add_executable(main-entry-hil-maker-faire src/entrypoints/Main/main-entry.cpp ${MAIN_COMPUTER} ${HIL})
target_include_directories(main-entry-hil-maker-faire PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-hil-maker-faire PRIVATE HILSimulation EUROC BUZZER_LOW USE_SERIAL_TRANSCEIVER INTERP DEATHSTACK_V2)
sbs_target(main-entry-hil-maker-faire stm32f429zi_skyward_death_stack_x_maker_faire)
# sbs_target(main-entry-hil-maker-faire stm32f429zi_stm32f4discovery)

add_executable(payload-entry-milano src/entrypoints/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-milano PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-milano PRIVATE MILANO)
sbs_target(payload-entry-milano stm32f429zi_skyward_death_stack_x)

add_executable(payload-entry-roccaraso src/entrypoints/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-roccaraso PRIVATE ROCCARASO)
sbs_target(payload-entry-roccaraso stm32f429zi_skyward_death_stack_x)

add_executable(payload-entry-euroc src/entrypoints/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-euroc PRIVATE EUROC)
sbs_target(payload-entry-euroc stm32f429zi_skyward_death_stack_x)

add_executable(ciuti-entry src/entrypoints/Ciuti/ciuti-entry.cpp ${CIUTI_COMPUTER})
target_include_directories(ciuti-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(ciuti-entry stm32f205rc_skyward_ciuti)

add_executable(payload-entry-hil src/entrypoints/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER} ${HIL})
target_include_directories(payload-entry-hil PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-hil PRIVATE PAYLOAD_ENTRY HILSimulation EUROC USE_SERIAL_TRANSCEIVER)
sbs_target(payload-entry-hil stm32f429zi_skyward_death_stack_x)

add_executable(groundstation-entry src/entrypoints/Groundstation/groundstation-entry.cpp)
target_include_directories(groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(groundstation-entry stm32f429zi_skyward_groundstation_v2)

#-----------------------------------------------------------------------------#
#                                    HIL Tests                                #
#-----------------------------------------------------------------------------#

add_executable(test-hil src/tests/hardware_in_the_loop/test-hil/test-hil.cpp ${MAIN_COMPUTER} ${HIL})
target_include_directories(test-hil PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(test-hil PRIVATE HILSimulation HILUseADA HILUseRadio EUROC)
# sbs_target(test-hil stm32f407vg_stm32f4discovery)
sbs_target(test-hil stm32f429zi_skyward_death_stack_v3)

#-----------------------------------------------------------------------------#
#                                Main Computer                                #
#-----------------------------------------------------------------------------#

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
sbs_target(main-test-servos stm32f429zi_skyward_death_stack_x_maker_faire)

add_executable(test-airbrakes-st src/tests/Main/actuators/test-airbrakes-st.cpp ${MAIN_COMPUTER} ${HIL})
target_include_directories(test-airbrakes-st PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(test-airbrakes-st PRIVATE HILSimulation)
sbs_target(test-airbrakes-st stm32f429zi_skyward_death_stack_x_maker_faire)

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

add_executable(main-test-vn100 src/tests/Main/sensors/test-vn100.cpp)
sbs_target(main-test-vn100 stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-ubxgps src/tests/Main/sensors/test-ubxgps.cpp)
sbs_target(main-test-ubxgps stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-leds src/tests/Main/test-leds.cpp)
sbs_target(main-test-leds stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-can-handler src/tests/Main/test-can-handler.cpp ${MAIN_COMPUTER})
target_include_directories(main-test-can-handler PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-can-handler stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-nas src/tests/Main/test-nas.cpp)
sbs_target(main-test-nas stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-radio src/tests/Main/test-radio.cpp ${MAIN_COMPUTER})
target_include_directories(main-test-radio PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-radio stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-sensors src/tests/Main/test-sensors.cpp ${MAIN_COMPUTER})
target_include_directories(main-test-sensors PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(main-test-sensors stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-serial-ports src/tests/Main/test-serial-ports.cpp)
sbs_target(main-test-serial-ports stm32f429zi_skyward_death_stack_v3)

add_executable(main-test-xbee src/tests/Main/test-xbee.cpp)
sbs_target(main-test-xbee stm32f429zi_skyward_death_stack_v3)

#-----------------------------------------------------------------------------#
#                               Payload Computer                              #
#-----------------------------------------------------------------------------#

add_executable(payload-test-nas src/tests/Payload/test-nas.cpp)
sbs_target(payload-test-nas stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#                              Auxiliary Computer                             #
#-----------------------------------------------------------------------------#

# add_executable(aux-test-can src/tests/Auxiliary/test-can.cpp)
# sbs_target(aux-test-can stm32f429zi_skyward_pyxis_auxiliary)

add_executable(aux-test-per src/tests/Auxiliary/test-auxiliary-peripherals.cpp)
sbs_target(aux-test-per stm32f429zi_skyward_pyxis_auxiliary)

#-----------------------------------------------------------------------------#
#                            Groundstation Computer                           #
#-----------------------------------------------------------------------------#

add_executable(groundstation-parafoil-entry src/entrypoints/Groundstation/groundstation-parafoil-entry.cpp)
sbs_target(groundstation-parafoil-entry stm32f429zi_skyward_groundstation_parafoil)

#-----------------------------------------------------------------------------#
#                                CIUTI Computer                               #
#-----------------------------------------------------------------------------#

add_executable(ciuti-test-ina188 src/tests/Ciuti/test-ina188.cpp)
sbs_target(ciuti-test-ina188 stm32f205rc_skyward_ciuti)

add_executable(ciuti-test-lis331 src/tests/Ciuti/test-lis331.cpp)
sbs_target(ciuti-test-lis331 stm32f205rc_skyward_ciuti)

add_executable(ciuti-test-usarts src/tests/Ciuti/test-usarts.cpp)
sbs_target(ciuti-test-usarts stm32f205rc_skyward_ciuti)
