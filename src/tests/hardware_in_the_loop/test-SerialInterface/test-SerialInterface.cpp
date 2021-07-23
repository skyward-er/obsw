/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>

#include "hardware_in_the_loop/HILConfig.h"
#include "hardware_in_the_loop/simulator_communication/SerialInterface.h"
#include "math/Vec3.h"

using namespace miosix;

int main()
{
    SerialInterface simulation(115200);
    SimulatorData sensorData;
    ActuatorData data{10};

    if (!simulation.init())
    {
        TRACE("Wrong initialization\n");
        return -1;
    }

    for (;;)
    {
        simulation.sendData<ActuatorData>(&data);
        simulation.recvData<SimulatorData>(&sensorData);
    }

    return 0;
}

/*
baudrate: 256000, avarage time: 0.0175
structToSingles avg_time: 0.0003
*/