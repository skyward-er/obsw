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
)

# set(MAIN_COMPUTER
# src/boards/MainComputer/events/EventStrings.cpp
# src/boards/MainComputer/Actuators/Actuators.cpp
# src/boards/MainComputer/AirBrakes/AirBrakesController.cpp
# src/boards/MainComputer/ApogeeDetectionAlgorithm/ADAController.cpp
# src/boards/MainComputer/Deployment/DeploymentController.cpp
# src/boards/MainComputer/NavigationAttitudeSystem/NASController.cpp
# src/boards/MainComputer/FlightStatsRecorder/FSRController.cpp
# )
set(PARAFOIL_MOCKUP_COMPUTER
    src/boards/Parafoil/Main/Sensors.cpp
    src/boards/Parafoil/Main/Radio.cpp
    src/boards/Parafoil/TelemetriesTelecommands/TMRepository.cpp
    src/boards/Parafoil/Wing/WingServo.cpp
    src/boards/Parafoil/Wing/WingAlgorithm.cpp
    src/boards/Parafoil/Wing/WingController.cpp
    src/boards/Parafoil/FlightModeManager/FMMController.cpp
    src/boards/Parafoil/Control/Algorithms.cpp
    src/boards/Parafoil/Control/NASController.cpp
)
set(PAYLOAD_COMPUTER
    src/boards/Payload/Main/Sensors.cpp
    src/boards/Payload/Main/Radio.cpp
    src/boards/Payload/TelemetriesTelecommands/TMRepository.cpp
    src/boards/Payload/Control/NASController.cpp
)
