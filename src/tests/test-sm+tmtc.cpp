
/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#include <TMTCManager/TMTCManager.h>
#include <events/Events.h>
#include <events/Topics.h>
#include <interfaces-impl/hwmapping.h>

#include "ADA/ADAController.h"
#include "Common.h"
#include "LoggerService/LoggerService.h"
#include "SensorManager/SensorManager.h"
#include "diagnostic/CpuMeter.h"
#include "events/EventBroker.h"

using namespace miosix;
using namespace DeathStackBoard;

int main()
{
    printf("** Sensor manager & TMTC test **\n");
    i2c1::init();
    busSPI2::init();
    /******************
     * SENSOR MANAGER *
     ******************/
    Stats s;
    ADAController ada;
    SensorManager mgr{&ada};
    // ada.start();
    // try
    // {
    //     LoggerService::getInstance()->start();
    // }
    // catch (const std::exception& e)
    // {
    //     printf("SDCARD MISSING\n");;
    //     for (;;)
    //     {
    //         led1::high();
    //         Thread::sleep(200);
    //         led1::low();
    //         Thread::sleep(200);
    //     }
    // }
    // led1::high();

    sEventBroker->start();
    mgr.start();

    sEventBroker->post({EV_TC_START_SENSOR_LOGGING}, TOPIC_TC);

    printf("Wait for calibration to complete.\n");

    Thread::sleep(500);

    /******************
     *  TMTC MANAGER  *
     ******************/

    TMTCManager* tmtc = new TMTCManager();
    tmtc->start();
    sEventBroker->start();

    Thread::sleep(1000);

    // printf("\nOk, press open to post liftoff...\n");
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