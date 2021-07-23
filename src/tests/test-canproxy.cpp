/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <Common.h>
#include <events/EventBroker.h>
#include <miosix.h>

#include "CanInterfaces.h"
#include "Canbus/CanProxy.h"

using namespace miosix;
using namespace CanInterfaces;
using namespace DeathStackBoard;

int main()
{
    CanManager* can_mgr = new CanManager(CAN1);
    CanProxy* can       = new CanProxy(can_mgr);

    sEventBroker->start();

    const char* pkt = "TestMSG";

    while (1)
    {
        TRACE("[CAN] Sending \n");
        ledOn();
        can->send(CAN_TOPIC_IGNITION, (const uint8_t*)pkt, strlen(pkt));
        // socket.receive(buf, 64);
        Thread::sleep(50);
        ledOff();

        Thread::sleep(500);
    }
}