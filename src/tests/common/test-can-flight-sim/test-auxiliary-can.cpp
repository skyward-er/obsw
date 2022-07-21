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
#include <common/canbus/MockSensors/MockAirBrakes.h>
#include <common/canbus/MockSensors/MockPitot.h>
#include <time.h>
#include <utils/collections/IRQCircularBuffer.h>

#include <thread>

#include "test-flight-event-handler.h"

using namespace Boardcore::Canbus;
using namespace miosix;
using namespace common;

CanHandler* handler;
bool running;
void receivePressure(MockPitot* pitot)
{
    while (true)
    {
        (*pitot).waitTillUpdated();
        (*pitot).getData();
        TRACE("ERROR received a pressure \n");
    }
}
void receiveAir(MockAirBrakes* air)
{
    int counter = 0;
    while (true)
    {
        while (running)
        {
            clock_t start = clock();
            for (int i = 0; i < nAir; i++)
            {
                (*air).waitTillUpdated();
                counter++;
                (*air).getData();
            }
            float time = float(clock() - start) / CLOCKS_PER_SEC;
            TRACE("received %d packets in %f second\n", nAir, time);
        }
        while (!running)
        {
            Thread::sleep(10);
        }
    }
}

int main()
{
    // We accept only packet with source main
    Filter f;
    f.source = Boards::Main;

    MockPitot* pitot         = new MockPitot(Pitot);
    MockAirBrakes* airBrakes = new MockAirBrakes(AirBrakes);
    handler                  = new CanHandler(Boards::Main);
    handler->addFilter(f);
    (*handler).addMock(airBrakes);
    (*handler).startHandler();

    // We expect to receive multiple*100 packet of AereoBrakes packet,and
    // multiple of each command packet

    MyEventHandler evh;
    TRACE("Start sending\n");
    if (evh.start())
    {
        std::thread recAir(receiveAir, airBrakes);
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
        evh.stop();  // it posts an EV_EMPTY to wake up the thread
    }
    else
    {
        TRACE("Failed to start MyEventHandler\n");
    }
}
