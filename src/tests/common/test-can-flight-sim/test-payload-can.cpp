/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <common/canbus/CanHandler.h>
#include <common/canbus/MockSensors/MockPitot.h>
#include <utils/collections/IRQCircularBuffer.h>

#include <atomic>
#include <thread>

#include "test-flight-event-handler.h"

using namespace Boardcore::Canbus;
using namespace miosix;
using namespace common;
CanHandler* handler;
std::atomic<bool> running;  // 1 element we are active 0 we are inactive
void receivePressure(MockPitot* pitot)
{
    while (true)
    {
        (*pitot).waitTillUpdated();
        (*pitot).getData();
        TRACE("ERROR received a pressure packet\n");
    }
}

void sendPressure(MockPitot* pitot)
{
    CanData temp = pitot->parseData({240, 1234.23});
    int f;
    int msWait = 1000 / (nPressure);
    while (true)
    {
        for (f = 0; f < nPressure && running; f++)
        {
            (*handler).sendCan(Boards::Main, common::Priority::Critical,
                               Type::Sensor, SensorId::Pitot, temp);
            Thread::sleep(msWait);
        }
        TRACE("Sent %d pressure packets\n", f);
        if (f == nPressure)
        {
            while (running)
            {
                Thread::sleep(10);
            }
        }
        while (!running)
        {
            Thread::sleep(10);
        }
    }
}

int main()
{

    // We accept only packet with source main and destination payload

    Filter f;
    f.source         = Boards::Main;
    f.destination    = Boards::Broadcast;
    MockPitot* pitot = new MockPitot(Pitot);
    handler          = new CanHandler(Boards::Main);
    handler->addFilter(f);

    (*handler).startHandler();

    // We expect to send multiple*100 packet of Pressure packet and receive
    // multiple of each command packet
    MyEventHandler evh;
    if (evh.start())
    {
        std::thread sendPress(sendPressure, pitot);
        std::thread recPress(receivePressure, pitot);
        for (;;)
        {
            running = true;
            evh.waitReset();  // First reset is to start sending
            evh.waitReset();  // the second to stop
            running = false;
            evh.checkCounters(nEvents);
            // We stop for 100 ms to wait for any unfinished print
            Thread::sleep(100);
        }
        evh.stop();  // It posts an EV_EMPTY to wake up the thread
    }
    else
    {
        TRACE("Failed to start MyEventHandler\n");
    }
}
