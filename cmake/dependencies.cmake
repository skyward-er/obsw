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
    src/boards/Main/BoardScheduler.cpp
    src/boards/Main/Sensors/Sensors.cpp
    src/boards/Main/StateMachines/NASController/NASController.cpp
    src/boards/Main/Radio/Radio.cpp
    src/boards/Main/TMRepository/TMRepository.cpp
    src/boards/Main/CanHandler/CanHandler.cpp
    src/boards/Main/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/boards/Main/Actuators/Actuators.cpp
    src/boards/Main/Sensors/RotatedIMU/RotatedIMU.cpp
    src/boards/Main/StateMachines/ADAController/ADAController.cpp
    src/boards/Main/PinHandler/PinHandler.cpp
    src/boards/Main/AltitudeTrigger/AltitudeTrigger.cpp
    src/boards/Main/StateMachines/ABKController/ABKController.cpp
    src/boards/Main/StateMachines/MEAController/MEAController.cpp
    src/boards/Main/StateMachines/Deployment/Deployment.cpp
    src/boards/Main/FlightStatsRecorder/FlightStatsRecorder.cpp
)

set(GROUNDSTATION_COMMON
    src/boards/Groundstation/Base/Radio/Radio.cpp
    src/boards/Groundstation/Base/Ports/Ethernet.cpp
    src/boards/Groundstation/Base/BoardStatus.cpp
    src/boards/Groundstation/Base/Hub.cpp
)

set(GS_COMPUTER
    src/boards/Gs/Ports/Serial.cpp
    src/boards/Gs/Radio/Radio.cpp
    src/boards/Gs/Radio/RadioStatus.cpp
    src/boards/Gs/Hub.cpp
)

set(MOTOR_SOURCES
    src/boards/Motor/Actuators/Actuators.cpp
    src/boards/Motor/Sensors/Sensors.cpp
    src/boards/Motor/BoardScheduler.cpp
    src/boards/Motor/CanHandler/CanHandler.cpp
) 

set(RIG_COMPUTER
    src/boards/RIG/BoardScheduler.cpp
    src/boards/RIG/Sensors/Sensors.cpp
    src/boards/RIG/Actuators/Actuators.cpp
    src/boards/RIG/Radio/Radio.cpp
    src/boards/RIG/TMRepository/TMRepository.cpp
    src/boards/RIG/StateMachines/GroundModeManager/GroundModeManager.cpp
    src/boards/RIG/StateMachines/TARS1/TARS1.cpp
    src/boards/RIG/CanHandler/CanHandler.cpp
    src/boards/RIG/StatesMonitor/StatesMonitor.cpp
)

set(CON_RIG_COMPUTER
    src/boards/con_RIG/Buttons/Buttons.cpp
    src/boards/con_RIG/Radio/Radio.cpp
    src/boards/con_RIG/BoardScheduler.cpp
)

set(PAYLOAD_COMPUTER
    src/boards/Payload/Actuators/Actuators.cpp
    src/boards/Payload/CanHandler/CanHandler.cpp
    src/boards/Payload/FlightStatsRecorder/FlightStatsRecorder.cpp
    src/boards/Payload/Sensors/Sensors.cpp
    src/boards/Payload/BoardScheduler.cpp
    src/boards/Payload/PinHandler/PinHandler.cpp
    src/boards/Payload/Radio/Radio.cpp
    src/boards/Payload/TMRepository/TMRepository.cpp
    src/boards/Payload/StateMachines/NASController/NASController.cpp
    src/boards/Payload/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/boards/Payload/StateMachines/WingController/WingController.cpp
    src/boards/Payload/AltitudeTrigger/AltitudeTrigger.cpp
    src/boards/Payload/VerticalVelocityTrigger/VerticalVelocityTrigger.cpp
    src/boards/Payload/Wing/AutomaticWingAlgorithm.cpp
    src/boards/Payload/Wing/Guidance/EarlyManeuverGuidanceAlgorithm.cpp
    src/boards/Payload/Wing/FileWingAlgorithm.cpp
    src/boards/Payload/Wing/WingAlgorithm.cpp
    src/boards/Payload/Sensors/RotatedIMU/RotatedIMU.cpp
    src/boards/Payload/WindEstimationScheme/WindEstimation.cpp
)

set(GROUNDSTATION_BASE
    src/boards/Groundstation/Base/Radio/Radio.cpp
    src/boards/Groundstation/Base/Ports/Ethernet.cpp
    src/boards/Groundstation/Base/BoardStatus.cpp
    src/boards/Groundstation/Base/Hub.cpp
)

set(GROUNDSTATION_NOKIA
    src/boards/Groundstation/Nokia/Radio/Radio.cpp
    src/boards/Groundstation/Nokia/Hub.cpp
)

set(GS_COMPUTER
    src/boards/Gs/Ports/Serial.cpp
    src/boards/Gs/Radio/Radio.cpp
    src/boards/Gs/Radio/RadioStatus.cpp
    src/boards/Gs/Hub.cpp
)

set(GROUNDSTATION_BASE
    src/boards/Groundstation/Common/Ports/Serial.cpp
    src/boards/Groundstation/Common/Ports/EthernetBase.cpp
    src/boards/Groundstation/Common/Radio/RadioBase.cpp
    src/boards/Groundstation/Common/HubBase.cpp
)

set(GROUNDSTATION_AUTOMATED
    src/boards/Groundstation/Automated/Radio/Radio.cpp
    src/boards/Groundstation/Automated/Radio/RadioStatus.cpp
    src/boards/Groundstation/Automated/Hub.cpp
)

set(ANTENNAS
    src/boards/Groundstation/Automated/Actuators/Actuators.cpp
    src/boards/Groundstation/Automated/Sensors/Sensors.cpp
)