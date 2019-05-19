/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISIN\G
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <boards/DeathStack/TMTCManager/TMTCManager.h>

#include <boards/DeathStack/Events.h>
#include <boards/DeathStack/Topics.h>

#include <interfaces-impl/hwmapping.h>

#include "Common.h"
#include "DeathStack/ADA/ADA.h"
#include "DeathStack/LogProxy/LogProxy.h"
#include "DeathStack/SensorManager/SensorManager.h"
#include "diagnostic/CpuMeter.h"
#include "events/EventBroker.h"

using namespace miosix;
using namespace DeathStackBoard;

int main()
{
    i2c1::init();
    busSPI2::init();
    led2::low();
    /******************
     * SENSOR MANAGER *
     ******************/
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

    sEventBroker->post({EV_TC_START_LOGGING}, TOPIC_TC);

    DeploymentAltitudeEvent dpl_ev;
    dpl_ev.dplAltitude = 1000;
    dpl_ev.sig         = EV_TC_SET_DPL_ALTITUDE;
    // sEventBroker->post(dpl_ev, TOPIC_TC);

    printf("Wait for calibration to complete.\n");

    Thread::sleep(500);

    /******************
     *  TMTC MANAGER  *
     ******************/

    TMTCManager* tmtc = new TMTCManager();
    tmtc->start();
    sEventBroker->start();

    Thread::sleep(5000);

    //printf("\nOk, press open to post liftoff...\n");
    // while(inputs::btn_open::value())
    // {
    //     Thread::sleep(100);
    // }

    printf("Spamming HR_TM\n");

    Thread::sleep(100);

    sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

    for (;;)
    {
        led1::high();
        Thread::sleep(1000);
        led1::low();
        Thread::sleep(1000);
    }

    printf("Press close to reboot...\n");
    while (inputs::btn_close::value())
    {
        Thread::sleep(100);
    }

    miosix::reboot();

    return 0;
}