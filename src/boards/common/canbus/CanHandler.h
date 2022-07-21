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
#include <common/canbus/CanConfig.h>
#include <common/canbus/MockSensors/MockSensor.h>
#include <drivers/canbus/CanProtocol.h>
#include <drivers/canbus/Canbus.h>

namespace common
{

struct Filter
{
    int8_t source      = -1;
    int8_t destination = -1;
};

/**
 * @brief Canbus high level implementation.
 *
 * This class takes care of creating the ID of the CanData, sending it to
 * CanProtocol and receiving incoming packets calling the right events.
 */
class CanHandler : public Boardcore::ActiveObject
{
private:
    Boards source;
    Boardcore::Canbus::CanProtocol *can;
    MockSensor *sensors[NumberOfSensor];
    uint8_t numberOfObservedSensor = 0;
    miosix::FastMutex mutex;

public:
    /**
     * @brief Construct a new CanHandler object
     *
     * @param source: the Id of the sending board.
     * @param pitotInstance: An instance of MockPitot.
     * @param brakesInstance: An instance of MockAirBrakes
     */
    CanHandler(Filter filter, Boards source)
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
        Boardcore::Canbus::Mask32FilterBank filterBank(filterId, filterMask, 1,
                                                       1, 0, 0, 0);
        Boardcore::Canbus::CanbusDriver::AutoBitTiming bt;
        bt.baudRate    = BAUD_RATE;
        bt.samplePoint = SAMPLE_POINT;
        Boardcore::Canbus::CanbusDriver *canbus =
            new Boardcore::Canbus::CanbusDriver(CAN1, {}, bt);
        canbus->addFilter(filterBank);
        canbus->init();
        can          = new Boardcore::Canbus::CanProtocol(canbus);
        this->source = source;
        (*can).start();
    }

    bool addMock(MockSensor *newSensor)
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
    /**
     * @brief Calculate the id of the packet and sends it to CanProtocol.
     *
     * @param destination: The Id of the destination board.
     * @param p: The priority of the packet.
     * @param t: The type of the packet sent.
     * @param idT: The IDtype of the packet sent.
     * @param toSend: The packet to be sent.
     */
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

    /**
     * @brief Calls sendData when we want to send a event (without creating a
     * CanData).
     *
     * @param destination: The Id of the destination board.
     * @param p: The priority of the packet.
     * @param t: The type of the packet sent.
     * @param idT: The IDtype of the packet sent.
     */
    void sendCan(Boards destination, Priority p, Type t, uint8_t idT)
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
    /**
     * @brief Keeps listening on CanProtocol for new packets, when a new packet
     * is received we update data and send events as needed
     */
    void run() override
    {
        Boardcore::Canbus::CanData data;

        while (true)
        {
            (*can).waitBufferNotEmpty();
            if (!((*can).isBufferEmpty()))
            {

                data = (*can).getPacket();
                switch ((data.canId & Boardcore::Canbus::type) >>
                        Boardcore::Canbus::shiftType)
                {
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
                        uint8_t tempID =
                            (data.canId & Boardcore::Canbus::idType) >>
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
        }
    }
};

}  // namespace common
