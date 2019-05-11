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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <boards/DeathStack/TMTCManager/TMTCManager.h>
#include "DeathStack/XbeeInterrupt.h"
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace DeathStackBoard;

int main()
{
    busSPI2::init();

    // Enable SPI
    xbee::cs::low();
    Thread::sleep(10);
    xbee::cs::high();
    Thread::sleep(10);

    enableXbeeInterrupt();

    TMTCManager* tmtc = new TMTCManager();
    tmtc->start();
    sEventBroker->start();

    sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
    Thread::sleep(1000);

    while(1)
    {
        Thread::sleep(5000);
    }
}