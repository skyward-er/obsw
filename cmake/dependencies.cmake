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

set(GROUNDSTATION_BASE
    src/boards/Groundstation/Base/Radio/Radio.cpp
    src/boards/Groundstation/Base/Radio/RadioStatus.cpp
    src/boards/Groundstation/Base/Ports/Ethernet.cpp
    src/boards/Groundstation/Base/Hub.cpp
)

set(GROUNDSTATION_COMMON
    src/boards/Groundstation/Common/Ports/Serial.cpp
    src/boards/Groundstation/Common/Ports/EthernetBase.cpp
    src/boards/Groundstation/Common/Radio/RadioBase.cpp
    src/boards/Groundstation/Common/HubBase.cpp
)