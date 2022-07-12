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

#pragma once

#include <ActiveObject.h>
#include <common/canbus/MockSensors/MockAirBrakes.h>
#include <common/canbus/MockSensors/MockPitot.h>
#include <drivers/canbus/CanProtocol.h>
#include <drivers/canbus/Canbus.h>
#include <events/EventBroker.h>
#include <utils/collections/IRQCircularBuffer.h>

namespace common
{

enum CanEvent : uint8_t
{
    EV_LIFTOFF = Boardcore::EV_FIRST_CUSTOM,
    EV_APOGEE,
    EV_ARMED,
    EV_AIRBRAKES
};

enum CanTopics : uint8_t
{
    TOPIC_CAN_EVENTS,
    TOPIC_CAN_COMMAND  // Maybe we can make this a single topic? not sure
};

enum SensorID
{
    Pitot = 0x00
};

enum EventsId
{
    Liftoff = 0x00,
    Apogee  = 0x01,
    Armed   = 0x02
};

enum CommandsID
{
    AirBrakes = 0x00
};

enum Boards
{
    Main      = 0x00,
    Payload   = 0x01,
    Auxiliary = 0x02
};

enum Priority
{
    Critical = 0x00,
    High     = 0x01,
    Medium   = 0x02,
    Low      = 0x03
};

enum Type
{
    Events  = 0x00,
    Command = 0x01,
    Sensor  = 0x02

};

class CanHandler : public Boardcore::ActiveObject
{
private:
    Boards source;
    MockPitot *pitot;
    MockAirBrakes *brakes;
    Boardcore::Canbus::CanProtocol *can;

public:
    CanHandler(Boardcore::Canbus::CanbusDriver *canPhysical, Boards source,
               MockPitot *pitotInstance, MockAirBrakes *brakesInstance)
        : source(source)
    {
        can    = new Boardcore::Canbus::CanProtocol(canPhysical);
        pitot  = pitotInstance;
        brakes = brakesInstance;
        (*can).start();
    }

    void sendCan(Boards destination, Priority p, Type t, uint8_t idT,
                 Boardcore::Canbus::CanData toSend)
    {
        if (toSend.length > 0)
        {
            toSend.canId =
                ((p << Boardcore::Canbus::shiftPriority) &
                 Boardcore::Canbus::priority) |
                ((t << Boardcore::Canbus::shiftType) &
                 Boardcore::Canbus::type) |
                ((source << Boardcore::Canbus::shiftSource) &
                 Boardcore::Canbus::source) |
                ((destination << Boardcore::Canbus::shiftDestination) &
                 Boardcore::Canbus::destination) |
                ((idT << Boardcore::Canbus::shiftIdType) &
                 Boardcore::Canbus::idType);
            can->sendData(toSend);
        }
    }

    void sendCan(Boards destination, Priority p, Type t, uint8_t idT,
                 uint8_t payload)
    {
        // If we have to send a command it is easier to call the function using
        // a int

        Boardcore::Canbus::CanData toSend;
        toSend.length     = 1;
        toSend.payload[0] = 0;
        sendCan(destination, p, t, idT, toSend);
    }

    ~CanHandler() { (*can).~CanProtocol(); }

protected:
    void run() override
    {
        Boardcore::Canbus::CanData data;

        while (true)
        {
            (*can).waitBufferEmpty();
            if (!((*can).isBufferEmpty()))
            {
                data = (*can).getPacket();

                switch ((data.canId & Boardcore::Canbus::type) >>
                        Boardcore::Canbus::shiftType)
                {
                    case Command:
                        switch ((data.canId & Boardcore::Canbus::idType) >>
                                Boardcore::Canbus::shiftIdType)
                        {
                            case AirBrakes:
                                (*brakes).setData(data);
                                Boardcore::EventBroker::getInstance().post(
                                    Boardcore::Event{EV_AIRBRAKES},
                                    TOPIC_CAN_COMMAND);
                                break;
                        }
                        break;
                    case Events:
                        switch ((data.canId & Boardcore::Canbus::idType) >>
                                Boardcore::Canbus::shiftIdType)
                        {
                            case Liftoff:
                                Boardcore::EventBroker::getInstance().post(
                                    Boardcore::Event{EV_LIFTOFF},
                                    TOPIC_CAN_EVENTS);
                                break;
                            case Apogee:
                                Boardcore::EventBroker::getInstance().post(
                                    Boardcore::Event{EV_APOGEE},
                                    TOPIC_CAN_EVENTS);
                                break;
                            case Armed:
                                Boardcore::EventBroker::getInstance().post(
                                    Boardcore::Event{EV_ARMED},
                                    TOPIC_CAN_EVENTS);
                                break;
                        }
                        break;

                    case Sensor:
                        switch ((data.canId & Boardcore::Canbus::idType) >>
                                Boardcore::Canbus::shiftIdType)
                        {
                            case Pitot:
                                (*pitot).setData(data);
                                break;
                        }
                        break;
                }
            }
        }
    }
};

}  // namespace common
