/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include "ADA/ADAController.h"
#include "LoggerService/LoggerService.h"
#include "Radio/TMTCManager.h"
#include "SensorManager/SensorManager.h"
#include "diagnostic/CpuMeter.h"
#include "events/EventBroker.h"
#include "events/Events.h"

using namespace miosix;
using namespace DeathStackBoard;

int main()
{
    sEventBroker.start();

    Stats s;
    SensorManager mgr;

    Bus bus;
    Sensors sensors(*bus.spi1, new TaskScheduler());
    Radio radio(*bus.spi2);

    sensors.start();
    radio.start();

    Thread::sleep(500);

    for (int i = 0; i < 1 * 3 * 10; i++)
    {
        s.add(getCpuStats());
        Thread::sleep(100);
    }

    printf("CPU: %f%%, min: %f max: %f\n", s.getStats().mean,
           s.getStats().minValue, s.getStats().maxValue);
    LoggerService::getInstance().stop();
    printf("End\n");

    for (;;)
    {
        ledOn();
        Thread::sleep(1000);
        ledOff();
        Thread::sleep(1000);
    }
}
