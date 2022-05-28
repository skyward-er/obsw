/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/Actuators/Actuators.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

void print()
{
    auto sample = Sensors::getInstance().lis3mdl->getLastSample();
    printf("[%.2f] %f %f %f\n", sample.magneticFieldTimestamp / 1e6,
           sample.magneticFieldX, sample.magneticFieldY, sample.magneticFieldZ);
}

int main()
{
    // Initialize the servo outputs
    (void)Actuators::getInstance();

    // Start the sensors sampling
    Sensors::getInstance().start();

    // Start the radio
    Radio::getInstance().start();

    TaskScheduler scheduler;
    scheduler.addTask(print, 1000);
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}
