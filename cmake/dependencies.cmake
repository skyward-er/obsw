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
    src/hardware_in_the_loop
)

set(HIL
    src/hardware_in_the_loop/HIL/HILFlightPhasesManager.cpp
    src/hardware_in_the_loop/HIL/HILTransceiver.cpp
)

set(PAYLOAD_COMPUTER
    src/boards/Payload/Actuators/Actuators.cpp
    src/boards/Payload/CanHandler/CanHandler.cpp
    #src/boards/Payload/FlightStatsRecorder/FlightStatsRecorder.cpp
    src/boards/Payload/Sensors/Sensors.cpp
    src/boards/Payload/BoardScheduler.cpp
    src/boards/Payload/PinHandler/PinHandler.cpp
    src/boards/Payload/Radio/Radio.cpp
    src/boards/Payload/TMRepository/TMRepository.cpp
    src/boards/Payload/StateMachines/NASController/NASController.cpp
    src/boards/Payload/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/boards/Payload/StateMachines/WingController/WingController.cpp
    src/boards/Payload/AltitudeTrigger/AltitudeTrigger.cpp
    src/boards/Payload/Wing/AutomaticWingAlgorithm.cpp
    src/boards/Payload/Wing/Guidance/EarlyManeuverGuidanceAlgorithm.cpp
    src/boards/Payload/Wing/FileWingAlgorithm.cpp
    src/boards/Payload/Wing/WingAlgorithm.cpp
    src/boards/Payload/WindEstimationScheme/WindEstimation.cpp
    #src/boards/Payload/StateMachines/Deployment/Deployment.cpp
)
