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
#include <drivers/canbus/BusLoadEstimation.h>
#include <drivers/canbus/Canbus.h>
#include <utils/collections/CircularBuffer.h>

#include <thread>

#include "test-can-event-handler.h"

#define slp 1000

using namespace Boardcore::Canbus;
using namespace miosix;
using namespace common;

void checkPressureData(MockPitot* pitot)
{
    Thread::sleep(10);  // De-synchronize the sending from the receiving;

    while (true)
    {
        if ((*pitot).waitTillUpdated())
        {
            TRACE("Received pressure packet. Data: %f, timestamp %llu\n",
                  (*pitot).getData().pressure,
                  (*pitot).getData().pressureTimestamp);
        }
        else
        {
            TRACE("No Pressure packet received in this time slot\n");
        }

        Thread::sleep(slp);
    }
}

int main()
{
    MockPitot* pitot = new MockPitot(SensorId::Pitot);
    CanHandler handler(Boards::Main);
    handler.addMock(pitot);
    Filter f;  // empty filter to accept all incoming messages

    handler.addFilter(f);
    handler.startHandler();
    //  send event, data and command
    Boardcore::PressureData t{240, 12354.35};
    MyEventHandler evh;
    if (evh.start())
    {
        std::thread printPressureData(checkPressureData, pitot);

        for (;;)
        {
            TRACE("Sent a packet \n");
            handler.sendCan(Boards::Payload, common::Priority::Medium,
                            Type::Sensor, SensorId::Pitot,
                            (*pitot).parseData(t));
            // if we have to send a command we do not use a payload
            handler.sendCan(Boards::Main, common::Priority::Low, Type::Events,
                            EventsId::Liftoff);
            handler.sendCan(Boards::Main, common::Priority::Low, Type::Events,
                            EventsId::Apogee);
            handler.sendCan(Boards::Main, common::Priority::Low, Type::Events,
                            EventsId::Armed);
            Thread::sleep(slp);
        }
        evh.stop();  // it posts an EV_EMPTY to wake up the thread
    }
    else
    {
        TRACE("Failed to start MyEventHandler\n");
    }
}
