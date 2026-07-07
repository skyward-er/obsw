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

set(GROUNDSTATION_COMMON
    src/Groundstation/Common/Radio/RadioBase.cpp
    src/Groundstation/Common/Ports/EthernetBase.cpp
    src/Groundstation/Common/Ports/EthernetSniffer.cpp
    src/Groundstation/Common/Ports/EthernetUtils.cpp
    src/Groundstation/Common/Ports/Serial.cpp
    src/Groundstation/Common/HubBase.cpp
)

set(PARAFOIL_COMPUTER
    src/Parafoil/Actuators/Actuators.cpp
    src/Parafoil/FlightStatsRecorder/FlightStatsRecorder.cpp
    src/Parafoil/PinHandler/PinHandler.cpp
    src/Parafoil/Radio/MessageHandler.cpp
    src/Parafoil/Radio/Radio.cpp
    src/Parafoil/Sensors/Sensors.cpp
    src/Parafoil/StateMachines/FlightModeManager/FlightModeManager.cpp
    src/Parafoil/StateMachines/NASController/NASController.cpp
    src/Parafoil/StateMachines/WingController/WingController.cpp
    src/Parafoil/StateMachines/ADAController/ADAController.cpp
    src/Parafoil/AltitudeTrigger/AltitudeTrigger.cpp
    src/Parafoil/AltitudeTrigger/LandingFlare.cpp
    src/Parafoil/Wing/AutocodedWingAlgorithm.cpp
    src/Parafoil/Wing/AutomaticWingAlgorithm.cpp
    src/Parafoil/Wing/Guidance/EarlyManeuverGuidanceAlgorithm.cpp
    src/Parafoil/Wing/Guidance/ClosedLoopGuidanceAlgorithm.cpp
    src/Parafoil/Wing/FileWingAlgorithm.cpp
    src/Parafoil/Wing/WingAlgorithm.cpp
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
