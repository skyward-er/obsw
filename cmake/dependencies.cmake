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
    src/boards
    src/hardware_in_the_loop
    src/tests/hardware_in_the_loop
    skyward-boardcore
)

set(AUXILIARY_COMPUTER
    src/boards/Auxiliary/Actuators/Actuators.cpp
    src/boards/Auxiliary/CanHandler/CanHandler.cpp
)

set(MAIN_COMPUTER
    src/boards/Main/Actuators/Actuators.cpp
    src/boards/Main/CanHandler/CanHandler.cpp
    src/boards/Main/Sensors/Sensors.cpp
    src/boards/Main/PinHandler/PinHandler.cpp
    src/boards/Main/Radio/Radio.cpp
    src/boards/Main/TMRepository/TMRepository.cpp
    src/boards/Main/StateMachines/AirBrakesController/AirBrakesController.cpp
    src/boards/Main/StateMachines/ADAController/ADAController.cpp
    src/boards/Main/StateMachines/Deployment/Deployment.cpp
    src/boards/Main/StateMachines/NASController/NASController.cpp
    src/boards/Main/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/boards/Main/StateMachines/FlightStatsRecorder/FlightStatsRecorder.cpp
)

set(PAYLOAD_COMPUTER
    src/boards/Payload/Actuators/Actuators.cpp
    src/boards/Payload/CanHandler/CanHandler.cpp
    src/boards/Payload/NASController/NASController.cpp
    src/boards/Payload/Radio/Radio.cpp
    src/boards/Payload/Sensors/Sensors.cpp
    src/boards/Payload/PinHandler/PinHandler.cpp
    src/boards/Payload/TMRepository/TMRepository.cpp
    src/boards/Payload/Wing/AutomaticWingAlgorithm.cpp
    src/boards/Payload/Wing/FileWingAlgorithm.cpp
    src/boards/Payload/Wing/WingAlgorithm.cpp
    src/boards/Payload/Wing/WingController.cpp
    src/boards/Payload/FlightModeManager/FlightModeManager.cpp
    src/boards/Payload/Wing/AltitudeTrigger.cpp
)

set(PARAFOIL_COMPUTER
    src/boards/Parafoil/Actuators/Actuators.cpp
    src/boards/Parafoil/FlightModeManager/FlightModeManager.cpp
    src/boards/Parafoil/NASController/NASController.cpp
    src/boards/Parafoil/Radio/Radio.cpp
    src/boards/Parafoil/Sensors/Sensors.cpp
    src/boards/Parafoil/TMRepository/TMRepository.cpp
    src/boards/Parafoil/Wing/WingAlgorithm.cpp
    src/boards/Parafoil/Wing/AutomaticWingAlgorithm.cpp
    src/boards/Parafoil/Wing/WingController.cpp
)

set(HIL_TESTS

    # src/boards/Main/Radio/Radio.cpp
    # src/boards/Main/TMRepository/TMRepository.cpp
    src/hardware_in_the_loop/HIL/HILTransceiver.cpp
    src/hardware_in_the_loop/HIL/HILFlightPhasesManager.cpp
    src/boards/Main/Actuators/Actuators.cpp
    src/boards/Main/Sensors/HILSensors.cpp
    src/boards/Main/StateMachines/AirBrakesController/AirBrakesController.cpp
    src/boards/Main/StateMachines/ADAController/ADAController.cpp
    src/boards/Main/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/boards/Main/StateMachines/Deployment/Deployment.cpp
    src/boards/Main/StateMachines/NASController/NASController.cpp
)

set(MOCK_LOGGER skyward-boardcore/src/shared/mock-Logger.cpp)