/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Common.h"
#include "DeathStack/ADA/ADA.h"
#include "DeathStack/Events.h"
#include "DeathStack/LogProxy/LogProxy.h"
#include "DeathStack/SensorManager/SensorManager.h"
#include "diagnostic/CpuMeter.h"
#include "events/EventBroker.h"

using namespace miosix;
using namespace DeathStackBoard;

int main()
{
    Stats s;
    ADA ada;
    SensorManager mgr{&ada};
    // ada.start();
    try
    {
        LoggerProxy::getInstance()->start();
    }
    catch (const std::exception& e)
    {
        printf("SDCARD MISSING\n");
        led2::low();
        for (;;)
        {
            led1::high();
            Thread::sleep(200);
            led1::low();
            Thread::sleep(200);
        }
    }
    led2::high();

    sEventBroker->start();
    mgr.start();

    sEventBroker->post({EV_TC_START_SENSOR_LOGGING}, TOPIC_TC);


    // printf("Wait for calibration to complete.\n");

    Thread::sleep(500);

    // sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

    for (int i = 0; i < 60 * 3 * 10; i++)
    {
        s.add(averageCpuUtilization());
        Thread::sleep(100);
    }

    printf("CPU: %f%%, min: %f max: %f\n", s.getStats().mean,
           s.getStats().minValue, s.getStats().maxValue);
    LoggerProxy::getInstance()->stop();
    printf("End\n");

    for (;;)
    {
        led1::high();
        Thread::sleep(1000);
        led1::low();
        Thread::sleep(1000);
    }
}
