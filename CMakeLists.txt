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

project(SkywardOnBoardSoftware)

#-----------------------------------------------------------------------------#
#                                 Entrypoints                                 #
#-----------------------------------------------------------------------------#

add_executable(death-stack-x-entry
    src/entrypoints/death-stack-x-entry.cpp
    ${DEATHSTACK_NEW_SOURCES}
    ${MOCK_SENSORS_DATA_SOURCES}
)
target_include_directories(death-stack-x-entry PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(death-stack-x-entry PRIVATE FLIGHT EUROC)
sbs_target(death-stack-x-entry stm32f429zi_skyward_death_stack_x)

add_executable(death-stack-x-testsuite src/entrypoints/death-stack-x-testsuite.cpp ${DEATHSTACK_NEW_SOURCES})
target_include_directories(death-stack-x-testsuite PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(death-stack-x-testsuite stm32f429zi_skyward_death_stack_x)

#add_executable(death-stack-entry src/entrypoints/death-stack-entry.cpp ${DEATHSTACK_SOURCES})
#target_include_directories(death-stack-entry PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(death-stack-entry PRIVATE DEATH_STACK_1)
#sbs_target(death-stack-entry stm32f429zi_skyward_death_stack_x)

#add_executable(death-stack-testsuite src/entrypoints/death-stack-testsuite.cpp ${DEATHSTACK_SOURCES})
#target_include_directories(death-stack-testsuite PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(death-stack-testsuite PRIVATE DEATH_STACK_1)
#sbs_target(death-stack-testsuite stm32f429zi_skyward_death_stack_x)

#add_executable(windtunnel-entry src/entrypoints/windtunnel-entry.cpp ${DEATHSTACK_NEW_SOURCES})
#target_include_directories(windtunnel-entry PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(windtunnel-entry stm32f429zi_skyward_death_stack_x)

#add_executable(hil-entry
#    src/entrypoints/hardware_in_the_loop/hil-entry.cpp
#    ${FMM_SOURCES}
#    ${PINHANDLER_SOURCES}
#    ${ADA_SOURCES}
#    ${NAVIGATION_SYSTEM_SOURCES}
#    ${LOGSERVICE_SOURCES}
#    ${DEPLOYMENT_SOURCES}
#)
#target_include_directories(hil-entry PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(hil-entry PRIVATE HARDWARE_IN_THE_LOOP)
#sbs_target(hil-entry stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#                                Tests - Catch                                #
#-----------------------------------------------------------------------------#

#add_executable(catch-tests-entry
#    src/tests/catch/catch-tests-entry.cpp
#    ${DEATHSTACK_SOURCES}
#    ${HERMES_TESTS_SOURCES}
#    ${ADA_TEST_SOURCES}
#    ${MOCK_SENSORS_DATA_SOURCES}
#)
#target_include_directories(catch-tests-entry PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(catch-tests-entry PRIVATE DEATH_STACK_1)
#sbs_target(catch-tests-entry stm32f429zi_skyward_death_stack_x)

add_executable(tests-catch
    src/tests/catch/catch-tests-entry.cpp
    ${DEATHSTACK_NEW_SOURCES}
    ${TESTS_OBSW_SOURCES}
    ${ADA_TEST_SOURCES}
    ${MOCK_SENSORS_DATA_SOURCES}
)
target_include_directories(tests-catch PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(tests-catch stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#                               Tests - Drivers                               #
#-----------------------------------------------------------------------------#

#add_executable(test-imus src/tests/drivers/test-imus.cpp)
#target_include_directories(test-imus PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-imus PRIVATE DEATH_STACK_1)
#sbs_target(test-imus stm32f429zi_skyward_death_stack_x)

#add_executable(test-cutter src/tests/drivers/test-cutter.cpp)
#target_include_directories(test-cutter PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-cutter stm32f429zi_skyward_death_stack_x)

#add_executable(test-mavlink src/tests/drivers/test-mavlink.cpp ${TMTC_SOURCES} ${LOGSERVICE_SOURCES})
#target_include_directories(test-mavlink PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-mavlink PRIVATE TRACE_EVENTS DEATH_STACK_1)
#sbs_target(test-mavlink stm32f429zi_skyward_death_stack_x)

#add_executable(ledwave src/tests/ledwave.cpp)
#target_include_directories(ledwave PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(ledwave PRIVATE <compile>)
#sbs_target(ledwave stm32f429zi_skyward_death_stack_x)

add_executable(ramtest src/tests/ram_test/ramtest.cpp ${RAM_TEST_SOURCES})
target_include_directories(ramtest PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(ramtest PRIVATE __ENABLE_XRAM)
sbs_target(ramtest stm32f429zi_skyward_death_stack_x)

#add_executable(test-motor src/tests/drivers/test-motor.cpp ${DEPLOYMENT_SOURCES})
#target_include_directories(test-motor PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-motor stm32f429zi_skyward_death_stack_x)

#add_executable(test-servo src/tests/drivers/test-servo.cpp ${LOGSERVICE_SOURCES} ${AIRBRAKES_SOURCES})
#target_include_directories(test-servo PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-servo stm32f429zi_skyward_death_stack_x)

#add_executable(test-hse src/tests/test-hse.cpp)
#target_include_directories(test-hse PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-hse stm32f429zi_skyward_death_stack_x)

#add_executable(test-all-sensors src/tests/drivers/test-all-sensors.cpp)
#target_include_directories(test-all-sensors PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-all-sensors PRIVATE DEATH_STACK_1)
#sbs_target(test-all-sensors stm32f429zi_skyward_death_stack_x)

#add_executable(test-preassure-calib src/tests/drivers/test-pressure-calib.cpp)
#target_include_directories(test-preassure-calib PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-preassure-calib PRIVATE DEATH_STACK_1)
#sbs_target(test-preassure-calib stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#                             Tests - Components                              #
#-----------------------------------------------------------------------------#

#add_executable(test-canproxy src/tests/test-canproxy.cpp)
#target_include_directories(test-canproxy PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-canproxy stm32f429zi_skyward_death_stack_x)

add_executable(test-pinhandler src/tests/test-pinhandler.cpp ${PINHANDLER_SOURCES} ${LOGSERVICE_SOURCES})
target_include_directories(test-pinhandler PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-pinhandler stm32f429zi_skyward_death_stack_x)

#add_executable(test-logproxy src/tests/test-logproxy.cpp ${LOGSERVICE_SOURCES})
#target_include_directories(test-logproxy PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-logproxy PRIVATE DEATH_STACK_1)
#sbs_target(test-logproxy stm32f429zi_skyward_death_stack_x)

#add_executable(test-sensormanager src/tests/test-sensormanager.cpp ${LOGSERVICE_SOURCES} ${ADA_TEST_SOURCES})
#target_include_directories(test-sensormanager PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-sensormanager PRIVATE USE_MOCK_SENSORS DEATH_STACK_1)
#sbs_target(test-sensormanager stm32f429zi_skyward_death_stack_x)

# Cannot use this as it is since the DeathStack singleton is required for handling messages in the TCHandler class
#add_executable(
#    test-sm+tmtc src/tests/test-sm+tmtc.cpp
#    ${LOGSERVICE_SOURCES}
#    ${ADA_SOURCES}
#    ${TMTC_SOURCES}
#    ${EVT_FUNCTIONS_SOURCES}
#    ${ADA_TEST_SOURCES}
#)
#target_include_directories(test-sm+tmtc PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-sm+tmtc PRIVATE DEATH_STACK_1)
#sbs_target(test-sm+tmtc stm32f429zi_skyward_death_stack)

#add_executable(test-fmm src/tests/catch/fsm/test-fmm.cpp ${DEATHSTACK_NEW_SOURCES})
#target_include_directories(test-fmm PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-fmm PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-fmm stm32f429zi_skyward_death_stack_x)

#add_executable(test-fmm-interactive src/tests/test-fmm-interactive.cpp ${DEATHSTACK_NEW_SOURCES})
#target_include_directories(test-fmm-interactive PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-fmm-interactive stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#                                 Tests - FSM                                 #
#-----------------------------------------------------------------------------#

#add_executable(test-ada
#    src/tests/catch/fsm/test-ada.cpp
#    ${ADA_SOURCES}
#    ${TEST_UTILS_SOURCES}
#    ${ADA_TEST_SOURCES}
#    ${MOCK_SENSORS_DATA_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-ada PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-ada PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-ada stm32f429zi_skyward_death_stack_x)

#add_executable(test-ada-simulation
#    src/tests/catch/ada/ada_kalman_p/test-ada-simulation.cpp
#    ${ADA_TEST_SOURCES}
#    ${DEATHSTACK_NEW_SOURCES}
#)
#target_include_directories(test-ada-simulation PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-ada-simulation PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-ada-simulation stm32f429zi_skyward_death_stack_x)

add_executable(test-ada-dpl-simulation
    src/tests/test-ada-dpl-simulation.cpp
    ${ADA_TEST_SOURCES}
    ${DEATHSTACK_NEW_SOURCES}
)
target_include_directories(test-ada-dpl-simulation PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-ada-dpl-simulation stm32f429zi_skyward_death_stack_x)

add_executable(test-nas
    src/tests/catch/fsm/test-nas.cpp
    ${NAVIGATION_SYSTEM_SOURCES}
    ${TEST_UTILS_SOURCES}
    ${MOCK_SENSORS_DATA_SOURCES}
    ${LOGSERVICE_SOURCES}
)
target_include_directories(test-nas PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(test-nas PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-nas stm32f429zi_skyward_death_stack_x)

add_executable(test-nas-simulation
    src/tests/catch/nas/test-nas-simulation.cpp
    ${NAVIGATION_SYSTEM_SOURCES}
    ${MOCK_SENSORS_DATA_SOURCES}
    ${LOGSERVICE_SOURCES}
)
target_include_directories(test-nas-simulation PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(test-nas-simulation PRIVATE STANDALONE_CATCH1_TEST)
sbs_target(test-nas-simulation stm32f429zi_skyward_death_stack_x)

#add_executable(test-tmtc src/tests/test-tmtc.cpp ${TEST_UTILS_SOURCES} ${DEATHSTACK_NEW_SOURCES})
#target_include_directories(test-tmtc PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-tmtc PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-tmtc stm32f429zi_skyward_death_stack_x)

#add_executable(test-flightstatsrecorder
#    src/tests/catch/fsm/test-flightstatsrecorder.cpp
#    ${TEST_UTILS_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-flightstatsrecorder PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-flightstatsrecorder PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-flightstatsrecorder stm32f429zi_skyward_death_stack_x)

#add_executable(test-ignition src/tests/catch/fsm/test-ignition.cpp ${TEST_UTILS_SOURCES})
#target_include_directories(test-ignition PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-ignition PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-ignition stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#                                 Tests - HIL                                 #
#-----------------------------------------------------------------------------#

#add_executable(test-SerialInterface src/tests/hardware_in_the_loop/test-SerialInterface/test-SerialInterface.cpp)
#target_include_directories(test-SerialInterface PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-SerialInterface PRIVATE HIL_SERIALINTERFACE)
#sbs_target(test-SerialInterface stm32f407vg_stm32f4discovery)

#add_executable(test-HIL src/tests/hardware_in_the_loop/test-HIL/test-HIL.cpp)
#target_include_directories(test-HIL PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-HIL PRIVATE HIL)
#sbs_target(test-HIL stm32f407vg_stm32f4discovery)

#add_executable(test-HIL+Aerobrake src/tests/hardware_in_the_loop/test-HIL+Aerobrake/test-HIL+Aerobrake.cpp)
#target_include_directories(test-HIL+Aerobrake PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-HIL+Aerobrake PRIVATE HIL_AEROBRAKE)
#sbs_target(test-HIL+Aerobrake stm32f407vg_stm32f4discovery)

#add_executable(test-HIL+AerobrakeController src/tests/hardware_in_the_loop/test-HIL+AerobrakeController/test-HIL+AerobrakeController.cpp)
#target_include_directories(test-HIL+AerobrakeController PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-HIL+AerobrakeController PRIVATE HIL_AEROBRAKE)
#sbs_target(test-HIL+AerobrakeController stm32f407vg_stm32f4discovery)

#add_executable(test-HIL+ADA
#    src/tests/hardware_in_the_loop/test-HIL+ApogeeDetectionAlgorithm/test-HIL+ADA.cpp
#    ${ADA_SOURCES}
#    ${ADA_TEST_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-HIL+ADA PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-HIL+ADA PRIVATE HIL_ADA)
#sbs_target(test-HIL+ADA stm32f429zi_skyward_death_stack_x)

#add_executable(test-HIL+ADA+Aerobrake
#    src/tests/hardware_in_the_loop/test-HIL+ADA+Aerobrake/test-HIL+ADA+Aerobrake.cpp
#    ${ADA_SOURCES}
#    ${ADA_TEST_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-HIL+ADA+Aerobrake PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-HIL+ADA+Aerobrake PRIVATE HIL_ADA_AEROBRAKE)
#sbs_target(test-HIL+ADA+Aerobrake stm32f407vg_stm32f4discovery)

#add_executable(test-HIL+ADA+AerobrakeController+nas
#    src/tests/hardware_in_the_loop/test-HIL+ADA+AerobrakeController+nas/test-HIL+ADA+AerobrakeController+nas.cpp
#    ${ADA_SOURCES}
#    ${ADA_TEST_SOURCES}
#    ${LOGSERVICE_SOURCES}
#    ${NAVIGATION_SYSTEM_SOURCES}
#)
#target_include_directories(test-HIL+ADA+AerobrakeController+nas PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-HIL+ADA+AerobrakeController+nas PRIVATE HIL_ADA_AEROBRAKECONTROLLER_NAS)
#sbs_target(test-HIL+ADA+AerobrakeController+nas stm32f429zi_stm32f4discovery)

#add_executable(eigen-test src/tests/eigen-test.cpp)
#target_include_directories(eigen-test PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(eigen-test stm32f429zi_stm32f4discovery)

#add_executable(test-mock-sensors src/tests/mock_sensors/test-mock-sensors.cpp ${MOCK_SENSORS_DATA})
#target_include_directories(test-mock-sensors PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-mock-sensors stm32f429zi_stm32f4discovery)

#add_executable(test-deployment
#    src/tests/catch/fsm/test-deployment.cpp
#    ${TEST_UTILS_SOURCES}
#    ${DEPLOYMENT_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-deployment PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-deployment PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-deployment stm32f429zi_skyward_death_stack_x)

#add_executable(test-deployment-interactive
#    src/tests/deployment/test-deployment-interactive.cpp
#    ${TEST_UTILS_SOURCES}
#    ${DEPLOYMENT_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-deployment-interactive PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-deployment-interactive PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-deployment-interactive stm32f429zi_skyward_death_stack_x)

#add_executable(test-airbrakes
#    src/tests/catch/fsm/test-airbrakes.cpp
#    ${TEST_UTILS_SOURCES}
#    ${LOGSERVICE_SOURCES}
#    ${AIRBRAKES_SOURCES}
#)
#target_include_directories(test-airbrakes PRIVATE ${OBSW_INCLUDE_DIRS})
#target_compile_definitions(test-airbrakes PRIVATE STANDALONE_CATCH1_TEST)
#sbs_target(test-airbrakes stm32f429zi_skyward_death_stack_x)

add_executable(test-airbrakes-interactive
    src/tests/airbrakes/test-airbrakes-interactive.cpp
    ${TEST_UTILS_SOURCES}
    ${LOGSERVICE_SOURCES}
    ${AIRBRAKES_SOURCES}
)
target_include_directories(test-airbrakes-interactive PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-airbrakes-interactive stm32f429zi_skyward_death_stack_x)

#add_executable(test-airbrakes-algorithm
#    src/tests/airbrakes/test-airbrakes-algorithm.cpp
#    ${LOGSERVICE_SOURCES}
#    ${AIRBRAKES_SOURCES}
#)
#target_include_directories(test-airbrakes-algorithm PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-airbrakes-algorithm stm32f429zi_skyward_death_stack_x)

#-----------------------------------------------------------------------------#
#         Tests - Comprehensive (included in death-stack-x-testsuite)         #
#-----------------------------------------------------------------------------#

#add_executable(test-power-board
#    src/tests/deathstack-boards/test-power-board.cpp
#    ${TEST_UTILS_SOURCES}
#    ${LOGSERVICE_SOURCES}
#    ${AIRBRAKES_SOURCES}
#)
#target_include_directories(test-power-board PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-power-board stm32f429zi_skyward_death_stack_x)

#add_executable(test-stm-board src/tests/deathstack-boards/test-stm-board.cpp)
#target_include_directories(test-stm-board PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-stm-board stm32f429zi_skyward_death_stack_x)

#add_executable(test-rf-board src/tests/deathstack-boards/test-rf-board.cpp)
#target_include_directories(test-rf-board PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-rf-board stm32f429zi_skyward_death_stack_x)

#add_executable(test-analog-board
#    src/tests/deathstack-boards/test-analog-board.cpp
#    ${PINHANDLER_SOURCES}
#    ${LOGSERVICE_SOURCES}
#)
#target_include_directories(test-analog-board PRIVATE ${OBSW_INCLUDE_DIRS})
#sbs_target(test-analog-board stm32f429zi_skyward_death_stack_x)
