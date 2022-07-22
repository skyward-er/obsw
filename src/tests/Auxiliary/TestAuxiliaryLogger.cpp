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
#include <drivers/canbus/CanProtocol.h>
#include <drivers/canbus/Canbus.h>

#include <functional>
#include <thread>

constexpr uint32_t BAUD_RATE = 500 * 1000;
constexpr float SAMPLE_POINT = 87.5f / 100.0f;

using std::string;
using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace miosix;
using namespace common;

#define SLP 100
miosix::FastMutex mutex;
MockPitot p(Pitot);

void print(CanData data)
{
    switch ((data.canId & Boardcore::Canbus::type) >>
            Boardcore::Canbus::shiftType)
    {
        case Events:
            switch ((data.canId & Boardcore::Canbus::idType) >>
                    Boardcore::Canbus::shiftIdType)
            {
                case Liftoff:
                    TRACE("Received event Liftoff\n");
                    break;
                case Apogee:
                    TRACE("Received event Apogee\n");
                    break;
                case Armed:
                    TRACE("Received event Armed\n");
                    break;
                default:
                    TRACE("Received uknown event id: %d\n",
                          ((data.canId & Boardcore::Canbus::idType) >>
                           Boardcore::Canbus::shiftIdType));
                    break;
            }
            break;

        case Sensor:
            uint8_t tempID = (data.canId & Boardcore::Canbus::idType) >>
                             Boardcore::Canbus::shiftIdType;
            switch (tempID)
            {
                case Pitot:
                    p.put(data);
                    PressureData pressure = p.getData();
                    TRACE(
                        "Received Pitot packet, pressure %f, timestamp %llu\n",
                        pressure.pressure, pressure.pressureTimestamp);
                    break;

                default:
                    TRACE("Uknown sensor id %d packet payload raw: ", tempID);
                    for (int i = 0; i < data.length; i++)
                    {
                        TRACE(" %llu ", data.payload[i]);
                    }
                    TRACE("\n");
                    break;
            }
            break;
    }
}

int main()
{
    CanbusDriver::CanbusConfig cfg{};
    CanbusDriver::AutoBitTiming bt;
    bt.baudRate     = common::BAUD_RATE;
    bt.samplePoint  = common::SAMPLE_POINT;
    CanbusDriver* c = new CanbusDriver(CAN1, cfg, bt);
    CanProtocol protocol(c, &print);
    // Allow every message
    Mask32FilterBank f2(0, 0, 1, 1, 0, 0, 0);

    c->addFilter(f2);
    c->init();
    protocol.start();

    TRACE("start \n");
    while (true)
    {
        Thread::sleep(10000);
        // TRACE("received packet \n");
    }
}
