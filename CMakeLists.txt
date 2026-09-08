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

cmake_minimum_required(VERSION 3.25)
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

sbs_declare_target(main-entry stm32f767zi_hydra_biscotto
    src/Main/main-entry.cpp
    ${MAIN_COMPUTER}
    DEFS DEFAULT_STDOUT_LOG_LEVEL=20)
if(TARGET main-entry)
    target_include_directories(main-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(main-entry-roccaraso stm32f767zi_hydra_biscotto
    src/Main/main-entry.cpp
    ${MAIN_COMPUTER}
    DEFS DEFAULT_STDOUT_LOG_LEVEL=20 ROCCARASO)
if(TARGET main-entry-roccaraso)
    target_include_directories(main-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(main-entry-euroc stm32f767zi_hydra_biscotto
    src/Main/main-entry.cpp
    ${MAIN_COMPUTER}
    DEFS DEFAULT_STDOUT_LOG_LEVEL=20 EUROC)
if(TARGET main-entry-euroc)
    target_include_directories(main-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(main-airbrakes-demo stm32f767zi_orion_biscotto
    src/Main/main-airbrakes-demo.cpp
    DEFS DEFAULT_STDOUT_LOG_LEVEL=20)
if(TARGET main-airbrakes-demo)
    target_include_directories(main-airbrakes-demo PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(payload-entry stm32f767zi_orion_biscotto
    src/Payload/payload-entry.cpp
    ${PAYLOAD_COMPUTER})
if(TARGET payload-entry)
    target_include_directories(payload-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(payload-entry-roccaraso stm32f767zi_orion_biscotto
    src/Payload/payload-entry.cpp
    ${PAYLOAD_COMPUTER}
    DEFS ROCCARASO)
if(TARGET payload-entry-roccaraso)
    target_include_directories(payload-entry-roccaraso PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(payload-entry-euroc stm32f767zi_orion_biscotto
    src/Payload/payload-entry.cpp
    ${PAYLOAD_COMPUTER}
    DEFS EUROC)
if(TARGET payload-entry-euroc)
    target_include_directories(payload-entry-euroc PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(motor-entry stm32f767zi_hydra_engine
    src/Motor/motor-entry.cpp
    ${MOTOR_SOURCES})
if(TARGET motor-entry)
    target_include_directories(motor-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(pitot-entry stm32f767zi_pitot
    src/Pitot/pitot-entry.cpp
    ${PITOT_COMPUTER})
if(TARGET pitot-entry)
    target_include_directories(pitot-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

#sbs_declare_target(rig-v2-entry stm32f767zi_rig_v2
#    src/RIGv2/rig-v2-entry.cpp
#    ${RIG_V2_COMPUTER})
#if(TARGET rig-v2-entry)
#    target_include_directories(rig-v2-entry PRIVATE ${OBSW_INCLUDE_DIRS})
#endif()

#sbs_declare_target(rig-v2-adc-test stm32f767zi_rig_v2
#    src/RIGv2/rig-v2-adc-test.cpp
#    ${RIG_V2_COMPUTER})
#if(TARGET rig-v2-adc-test)
#    target_include_directories(rig-v2-adc-test PRIVATE ${OBSW_INCLUDE_DIRS})
#endif()

sbs_declare_target(rig-v3-entry stm32f767zi_rig_v3
    src/RIGv3/rig-v3-entry.cpp
    ${RIG_V3_COMPUTER})
if(TARGET rig-v3-entry)
    target_include_directories(rig-v3-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(rig-v3-adc-test stm32f767zi_rig_v3
    src/RIGv3/rig-v3-adc-test.cpp
    ${RIG_V3_COMPUTER})
if(TARGET rig-v3-adc-test)
    target_include_directories(rig-v3-adc-test PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(con_rig-entry stm32f429zi_con_rig
    src/ConRIG/con_rig-entry.cpp
    ${CON_RIG_COMPUTER})
if(TARGET con_rig-entry)
    target_include_directories(con_rig-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(conrig-v2-entry stm32f767zi_conrig_v2
    src/ConRIGv2/conrig-v2-entry.cpp
    ${CONRIG_V2_COMPUTER})
if(TARGET conrig-v2-entry)
    target_include_directories(conrig-v2-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(rovie-groundstation-entry stm32f767zi_gemini_gs
    src/Groundstation/Rovie/rovie-groundstation-entry.cpp
    ${GROUNDSTATION_COMMON} ${GROUNDSTATION_ROVIE})
if(TARGET rovie-groundstation-entry)
    target_include_directories(rovie-groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(nokia-groundstation-entry stm32f429zi_nokia
    src/Groundstation/Nokia/nokia-groundstation-entry.cpp
    ${GROUNDSTATION_COMMON} ${GROUNDSTATION_NOKIA})
if(TARGET nokia-groundstation-entry)
    target_include_directories(nokia-groundstation-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()

sbs_declare_target(lyra-gs-entry stm32f767zi_lyra_gs
    src/Groundstation/LyraGS/lyra-gs-entry.cpp
    ${LYRA_GS} ${GROUNDSTATION_COMMON})
if(TARGET lyra-gs-entry)
    target_include_directories(lyra-gs-entry PRIVATE ${OBSW_INCLUDE_DIRS})
endif()
