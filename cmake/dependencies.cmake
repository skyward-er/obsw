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

set(OBSW_INCLUDE_DIRS
    src
    src/boards
)

set(MAIN_COMPUTER
    src/common/canbus/MotorStatus.cpp
    src/Main/Data/ABKTrajectorySet.cxx
    src/Main/PersistentVars/PersistentVars.cpp
    src/Main/HIL/HIL.cpp
    src/Main/Sensors/Sensors.cpp
    src/Main/AlgoReference/AlgoReference.cpp
    src/Main/Radio/Radio.cpp
    src/Main/CanHandler/CanHandler.cpp
    src/Main/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/Main/Actuators/Actuators.cpp
    src/Main/StateMachines/NASController/NASController.cpp
    src/Main/StateMachines/ADAController/ADAController.cpp
    src/Main/PinHandler/PinHandler.cpp
    src/Main/StateMachines/ABKController/ABKController.cpp
    src/Main/StateMachines/MEAController/MEAController.cpp
    src/Main/StatsRecorder/StatsRecorder.cpp
)

set(GROUNDSTATION_COMMON
    src/Groundstation/Common/Radio/RadioBase.cpp
    src/Groundstation/Common/Ports/EthernetBase.cpp
    src/Groundstation/Common/Ports/EthernetSniffer.cpp
    src/Groundstation/Common/Ports/Serial.cpp
    src/Groundstation/Common/HubBase.cpp
)

set(MOTOR_SOURCES
    src/Motor/PersistentVars/PersistentVars.cpp
    src/Motor/HIL/HIL.cpp
    src/Motor/Actuators/Actuators.cpp
    src/Motor/Sensors/Sensors.cpp
    src/Motor/CanHandler/CanHandler.cpp
)

set(RIG_V2_COMPUTER
    src/common/canbus/MotorStatus.cpp
    src/RIGv2/Radio/Radio.cpp
    src/RIGv2/Sensors/Sensors.cpp
    src/RIGv2/Actuators/Actuators.cpp
    src/RIGv2/Registry/Registry.cpp
    src/RIGv2/CanHandler/CanHandler.cpp
    src/RIGv2/StateMachines/GroundModeManager/GroundModeManager.cpp
    src/RIGv2/StateMachines/TARS1/TARS1.cpp
    src/RIGv2/StateMachines/TARS3/TARS3.cpp
)

set(CON_RIG_COMPUTER
    src/ConRIG/Buttons/Buttons.cpp
    src/ConRIG/Radio/Radio.cpp
    src/ConRIG/Serial/Serial.cpp
)

set(CONRIG_V2_COMPUTER
    src/ConRIGv2/Buttons/Buttons.cpp
    src/ConRIGv2/Radio/Radio.cpp
    src/ConRIGv2/Hub/Hub.cpp
    src/ConRIGv2/BoardStatus.cpp
)

set(PAYLOAD_COMPUTER
    src/Payload/Actuators/Actuators.cpp
    src/Payload/CanHandler/CanHandler.cpp
    src/Payload/FlightStatsRecorder/FlightStatsRecorder.cpp
    src/Payload/HIL/HIL.cpp
    src/Payload/Sensors/Sensors.cpp
    src/Payload/PersistentVars/PersistentVars.cpp
    src/Payload/PinHandler/PinHandler.cpp
    src/Payload/Radio/Radio.cpp
    src/Payload/Radio/MessageHandler.cpp
    src/Payload/StateMachines/NASController/NASController.cpp
    src/Payload/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/Payload/StateMachines/WingController/WingController.cpp
    src/Payload/AltitudeTrigger/AltitudeTrigger.cpp
    src/Payload/Wing/AutomaticWingAlgorithm.cpp
    src/Payload/Wing/Guidance/EarlyManeuverGuidanceAlgorithm.cpp
    src/Payload/Wing/Guidance/ClosedLoopGuidanceAlgorithm.cpp
    src/Payload/Wing/FileWingAlgorithm.cpp
    src/Payload/Wing/WingAlgorithm.cpp
)

set(GROUNDSTATION_ROVIE
    src/Groundstation/Rovie/Radio/Radio.cpp
    src/Groundstation/Rovie/Ports/Ethernet.cpp
    src/Groundstation/Rovie/Hub.cpp
)

set(GROUNDSTATION_NOKIA
    src/Groundstation/Nokia/Radio/Radio.cpp
    src/Groundstation/Nokia/Hub.cpp
)

set (LYRA_GS
    src/Groundstation/LyraGS/Radio/Radio.cpp
    src/Groundstation/LyraGS/Ports/Ethernet.cpp
    src/Groundstation/LyraGS/BoardStatus.cpp
    src/Groundstation/LyraGS/Base/Hub.cpp
    src/Groundstation/Automated/Hub.cpp
    src/Groundstation/Automated/Leds/Leds.cpp
    src/Groundstation/Automated/SMA/SMA.cpp
    src/Groundstation/Automated/Actuators/Actuators.cpp
    src/Groundstation/Automated/Sensors/Sensors.cpp
    src/Groundstation/Automated/PinHandler/PinHandler.cpp
    src/Groundstation/LyraGS/Ports/SerialLyraGS.cpp
)
