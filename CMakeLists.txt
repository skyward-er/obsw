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

# -----------------------------------------------------------------------------#
#                                    Project                                   #
# -----------------------------------------------------------------------------#

project(OnBoardSoftware)

include(skyward-boardcore/cmake/sbs.cmake)
include(cmake/dependencies.cmake)

#-----------------------------------------------------------------------------#
#                              Flight entrypoints                             #
#-----------------------------------------------------------------------------#

add_executable(main-entry src/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry PRIVATE DEFAULT_STDOUT_LOG_LEVEL=20)
sbs_target(main-entry stm32f767zi_orion_biscotto)

add_executable(main-entry-roccaraso src/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-roccaraso PRIVATE DEFAULT_STDOUT_LOG_LEVEL=20 ROCCARASO)
sbs_target(main-entry-roccaraso stm32f767zi_orion_biscotto)

add_executable(main-entry-euroc src/Main/main-entry.cpp ${MAIN_COMPUTER})
target_include_directories(main-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(main-entry-euroc PRIVATE DEFAULT_STDOUT_LOG_LEVEL=20 EUROC)
sbs_target(main-entry-euroc stm32f767zi_orion_biscotto)

add_executable(payload-entry src/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(payload-entry stm32f767zi_lyra_biscotto)

add_executable(payload-entry-roccaraso src/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-roccaraso PRIVATE ROCCARASO)
sbs_target(payload-entry-roccaraso stm32f767zi_lyra_biscotto)

add_executable(payload-entry-euroc src/Payload/payload-entry.cpp ${PAYLOAD_COMPUTER})
target_include_directories(payload-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(payload-entry-euroc PRIVATE EUROC)
sbs_target(payload-entry-euroc stm32f767zi_lyra_biscotto)

add_executable(motor-entry src/Motor/motor-entry.cpp ${MOTOR_SOURCES})
target_include_directories(motor-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(motor-entry stm32f767zi_lyra_motor)

add_executable(rig-v2-entry src/RIGv2/rig-v2-entry.cpp ${RIG_V2_COMPUTER})
target_include_directories(rig-v2-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(rig-v2-entry stm32f767zi_rig_v2)

add_executable(rig-v2-adc-test src/RIGv2/rig-v2-adc-test.cpp ${RIG_V2_COMPUTER})
target_include_directories(rig-v2-adc-test PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(rig-v2-adc-test stm32f767zi_rig_v2)

add_executable(con_rig-entry src/ConRIG/con_rig-entry.cpp ${CON_RIG_COMPUTER})
target_include_directories(con_rig-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(con_rig-entry stm32f429zi_con_rig)

add_executable(conrig-v2-entry src/ConRIGv2/conrig-v2-entry.cpp ${CONRIG_V2_COMPUTER})
target_include_directories(conrig-v2-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(conrig-v2-entry stm32f767zi_conrig_v2)

add_executable(rovie-groundstation-entry 
    src/Groundstation/Rovie/rovie-groundstation-entry.cpp 
    ${GROUNDSTATION_COMMON} ${GROUNDSTATION_ROVIE}
)
target_include_directories(rovie-groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(rovie-groundstation-entry stm32f767zi_gemini_gs)

add_executable(nokia-groundstation-entry 
    src/Groundstation/Nokia/nokia-groundstation-entry.cpp 
    ${GROUNDSTATION_COMMON} ${GROUNDSTATION_NOKIA}
)
target_include_directories(nokia-groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(nokia-groundstation-entry stm32f429zi_nokia)

add_executable(test-actuators
    src/Groundstation/Automated/test-actuators.cpp
    ${LYRA_GS} ${GROUNDSTATION_COMMON}
)
target_include_directories(test-actuators PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(test-actuators stm32f767zi_lyra_gs)

add_executable(lyra-gs-entry
    src/Groundstation/LyraGS/lyra-gs-entry.cpp
    ${LYRA_GS} ${GROUNDSTATION_COMMON}
)
target_include_directories(lyra-gs-entry PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(lyra-gs-entry stm32f767zi_lyra_gs)

#-----------------------------------------------------------------------------#
#                            Parafoil entrypoints                             #
#-----------------------------------------------------------------------------#

add_executable(parafoil-progressive-rotation src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-progressive-rotation PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-progressive-rotation PRIVATE ALGORITHM_PROGRESSIVE_ROTATION MILANO)
sbs_target(parafoil-progressive-rotation stm32f429zi_death_stack_v2)

add_executable(parafoil-guided-milano src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-guided-milano PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-guided-milano PRIVATE ALGORITHM_CLOSED_LOOP MILANO)
sbs_target(parafoil-guided-milano stm32f429zi_death_stack_v2)

add_executable(parafoil-guided-jesolo src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-guided-jesolo PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-guided-jesolo PRIVATE ALGORITHM_CLOSED_LOOP JESOLO)
sbs_target(parafoil-guided-jesolo stm32f429zi_death_stack_v2)

add_executable(parafoil-guided-dynamic src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-guided-dynamic PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-guided-dynamic PRIVATE ALGORITHM_CLOSED_LOOP DYNAMIC_TARGET
    DYNAMIC_TARGET_LATITUDE_OFFSET=35
    DYNAMIC_TARGET_LONGITUDE_OFFSET=-156
)
sbs_target(parafoil-guided-dynamic stm32f429zi_death_stack_v2)

add_executable(parafoil-t-approach-jesolo src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-t-approach-jesolo PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-t-approach-jesolo PRIVATE ALGORITHM_EARLY_MANEUVER JESOLO)
sbs_target(parafoil-t-approach-jesolo stm32f429zi_death_stack_v2)

add_executable(parafoil-t-approach-milano src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-t-approach-milano PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-t-approach-milano PRIVATE ALGORITHM_EARLY_MANEUVER MILANO)
sbs_target(parafoil-t-approach-milano stm32f429zi_death_stack_v2)

add_executable(parafoil-t-approach-dynamic src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-t-approach-dynamic PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-t-approach-dynamic PRIVATE
    ALGORITHM_EARLY_MANEUVER DYNAMIC_TARGET
    DYNAMIC_TARGET_LATITUDE_OFFSET=35
    DYNAMIC_TARGET_LONGITUDE_OFFSET=-156
)
sbs_target(parafoil-t-approach-dynamic stm32f429zi_death_stack_v2)

add_executable(parafoil-ground-test src/Parafoil/parafoil-entry.cpp ${PARAFOIL_COMPUTER})
target_include_directories(parafoil-ground-test PRIVATE ${OBSW_INCLUDE_DIRS})
target_compile_definitions(parafoil-ground-test PRIVATE ALGORITHM_PROGRESSIVE_ROTATION MILANO DISABLE_LANDING_FLARE)
sbs_target(parafoil-ground-test stm32f429zi_death_stack_v2)

add_executable(mockup-main src/MockupMain/mockup-entry.cpp ${MOCKUP_MAIN_COMPUTER})
target_include_directories(mockup-main PRIVATE ${OBSW_INCLUDE_DIRS})
sbs_target(mockup-main stm32f429zi_death_stack_v2)