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
#include "CanHandler.h"

namespace common
{
CanHandler::CanHandler(Boards source)
{

    Boardcore::Canbus::CanbusDriver::AutoBitTiming bt;
    bt.baudRate    = BAUD_RATE;
    bt.samplePoint = SAMPLE_POINT;
    canPhy         = new Boardcore::Canbus::CanbusDriver(CAN1, {}, bt);
    this->source   = source;
    can            = new Boardcore::Canbus::CanProtocol(
                   canPhy, std::bind(&CanHandler::callback, this, std::placeholders::_1));
}

void CanHandler::startHandler()
{
    canPhy->init();
    (*can).start();
}

bool CanHandler::addFilter(Filter filter)
{
    uint32_t filterId   = 0;
    uint32_t filterMask = 0;

    if (filter.source >= 0)
    {
        filterId =
            (filter.source
             << (Boardcore::Canbus::ShiftInformation::shiftSource +
                 Boardcore::Canbus::ShiftInformation::shiftSequentialInfo));
        filterMask =
            (Boardcore::Canbus::IDMask::source
             << Boardcore::Canbus::ShiftInformation::shiftSequentialInfo);
    }

    if (filter.destination >= 0)
    {
        filterId =
            filterId |
            (filter.destination
             << (Boardcore::Canbus::ShiftInformation::shiftDestination +
                 Boardcore::Canbus::ShiftInformation::shiftSequentialInfo));
        filterMask =
            filterMask |
            (Boardcore::Canbus::IDMask::destination
             << Boardcore::Canbus::ShiftInformation::shiftSequentialInfo);
    }
    Boardcore::Canbus::Mask32FilterBank filterBank(filterId, filterMask, 1, 1,
                                                   0, 0, 0);
    return canPhy->addFilter(filterBank);
}

bool CanHandler::addMock(MockSensor *newSensor)
{
    if (numberOfObservedSensor < SensorID::NumberOfSensor)
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        sensors[numberOfObservedSensor] = newSensor;
        numberOfObservedSensor++;
        return true;
    }
    return false;
}

void CanHandler::sendCan(Boards destination, Priority p, Type t, uint8_t idT,
                         Boardcore::Canbus::CanData toSend)
{
    if (toSend.length > 0)
    {
        toSend.canId =
            ((p << Boardcore::Canbus::shiftPriority) &
             Boardcore::Canbus::priority) |
            ((t << Boardcore::Canbus::shiftType) & Boardcore::Canbus::type) |
            ((source << Boardcore::Canbus::shiftSource) &
             Boardcore::Canbus::source) |
            ((destination << Boardcore::Canbus::shiftDestination) &
             Boardcore::Canbus::destination) |
            ((idT << Boardcore::Canbus::shiftIdType) &
             Boardcore::Canbus::idType);
        can->send(toSend);
    }
}

void CanHandler::sendCan(Boards destination, Priority p, Type t, uint8_t idT)
{
    // If we have to send a command it is easier to call the function using
    // a int

    Boardcore::Canbus::CanData toSend;
    toSend.length     = 1;
    toSend.payload[0] = 0;
    sendCan(destination, p, t, idT, toSend);
}

CanHandler::~CanHandler() { delete canPhy; }

void CanHandler::callback(Boardcore::Canbus::CanData data)
{
    switch ((data.canId & Boardcore::Canbus::type) >>
            Boardcore::Canbus::shiftType)
    {
        case Events:
            switch ((data.canId & Boardcore::Canbus::idType) >>
                    Boardcore::Canbus::shiftIdType)
            {
                case Liftoff:
                    Boardcore::EventBroker::getInstance().post(
                        Boardcore::Event{EV_LIFTOFF}, TOPIC_CAN_EVENTS);
                    break;
                case Apogee:
                    Boardcore::EventBroker::getInstance().post(
                        Boardcore::Event{EV_APOGEE}, TOPIC_CAN_EVENTS);
                    break;
                case Armed:
                    Boardcore::EventBroker::getInstance().post(
                        Boardcore::Event{EV_ARMED}, TOPIC_CAN_EVENTS);
                    break;
            }
            break;

        case Sensor:
            uint8_t tempID = (data.canId & Boardcore::Canbus::idType) >>
                             Boardcore::Canbus::shiftIdType;
            for (int i = 0; i < numberOfObservedSensor; i++)
            {
                miosix::Lock<miosix::FastMutex> l(mutex);
                if (sensors[i]->getID() == tempID)
                {
                    sensors[i]->put(data);
                }
            }
            break;
    }
}
}  // namespace common
