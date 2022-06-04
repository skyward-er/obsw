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
#include <Main/BoardScheduler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

void print()
{
    auto status = Radio::getInstance().mavDriver->getStatus();

    printf("[%.2f] %d\n", status.timestamp / 1e6,
           status.mavStats.packet_rx_success_count);
}

int main()
{
    // Initialize the servo outputs
    (void)Actuators::getInstance();

    // Start the sensors sampling
    Sensors::getInstance().start();

    // Start the radio
    Radio::getInstance().start();

    // Start the board task scheduler
    BoardScheduler::getInstance().getScheduler().start();

    TaskScheduler scheduler;
    scheduler.addTask(print, 1000);
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}
