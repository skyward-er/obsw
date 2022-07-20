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

#include <atomic>
#include <thread>

#include "test-flight-event-handler.h"

using namespace Boardcore::Canbus;
using namespace miosix;
using namespace common;

CanHandler* handler;
std::atomic<bool> running;  // 1 element we are active 0 we are inactive
miosix::FastMutex mutex;
void receivePressure(MockPitot* pitot)
{
    int counter = 0;
    while (true)
    {
        clock_t start = clock();
        for (int i = 0; i < nPressure; i++)
        {
            (*pitot).waitTillUpdated();
            counter++;
            (*pitot).getData();
        }
        float time = float(clock() - start) / CLOCKS_PER_SEC;
        TRACE("received %d packets in %f second\n", nPressure, time);
    }
}

void receiveAir(MockAirBrakes* air)
{
    while (true)
    {
        (*air).waitTillUpdated();
        (*air).getData();
        TRACE("ERROR received an AirBrake packet\n");
    }
}

void sendAirBrakes(MockAirBrakes* airbrakes)
{
    int f;
    int msWait = 1000 / (nAir);
    while (true)
    {
        for (f = 0; f < nAir && running; f++)
        {
            {
                miosix::Lock<miosix::FastMutex> l(mutex);
                (*handler).sendCan(
                    Boards::Auxiliary, common::Priority::Critical, Type::Sensor,
                    SensorID::AirBrakes, (*airbrakes).parseData(69));
            }
            Thread::sleep(msWait);
        }
        TRACE("Sent %d airbrakes packets\n", f);
        if (f == nAir)
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

void sendCommands()
{
    int f;
    int msWait = 1000 / (nEvents * 3);
    while (true)
    {
        for (f = 0; f < nEvents && running; f++)
        {
            {
                miosix::Lock<miosix::FastMutex> l(mutex);
                (*handler).sendCan(Boards::Broadcast, common::Priority::High,
                                   Type::Events, EventsId::Liftoff);
            }
            Thread::sleep(msWait);
            {
                miosix::Lock<miosix::FastMutex> l(mutex);
                (*handler).sendCan(Boards::Broadcast, common::Priority::Low,
                                   Type::Events, EventsId::Apogee);
            }
            Thread::sleep(msWait);
            {
                miosix::Lock<miosix::FastMutex> l(mutex);
                (*handler).sendCan(Boards::Broadcast, common::Priority::Low,
                                   Type::Events, EventsId::Armed);
            }
            Thread::sleep(msWait);
        }
        TRACE("Sent %d commands\n", f);
        if (f == nEvents)
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
    // We accept only packet with destination main
    Filter f;
    f.destination            = Boards::Main;
    MockPitot* pitot         = new MockPitot(Pitot);
    MockAirBrakes* airBrakes = new MockAirBrakes(AirBrakes);
    handler                  = new CanHandler(f, Boards::Main);
    (*handler).addMock(pitot);

    (*handler).start();

    // We expect to receive multiple*100 packet of Pressure packet,
    // send multiple*100 AirBrakes Packet and multiple of each command packet
    MyEventHandler evh;
    TRACE("Start sending\n");
    if (evh.start())
    {
        std::thread recPress(receivePressure, pitot);
        std::thread recAir(receiveAir, airBrakes);
        std::thread sendAir(sendAirBrakes, airBrakes);
        std::thread sendCmd(sendCommands);
        for (;;)
        {
            running = true;
            Thread::sleep(1000);
            running = false;
            evh.checkCounters(0);
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
