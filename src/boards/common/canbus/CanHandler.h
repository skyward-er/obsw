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
#include <drivers/canbus/CanProtocol.h>
#include <drivers/canbus/Canbus.h>
#include <utils/collections/IRQCircularBuffer.h>

namespace common
{
// sensors
enum SensorID
{
    Pitot = 0x00
};

// events
enum EventsId
{
    Liftoff = 0x00,
    Apogee  = 0x01,
    Armed   = 0x02
};

// commands
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

Boards source;
Boards destination;

class CanHandler : public Boardcore::ActiveObject
{
private:
    Boardcore::IRQCircularBuffer<Boardcore::Canbus::CanData, NPACKET> buffer;
    Boards source;
    Boardcore::Canbus::CanProtocol *can;

public:
    CanHandler(Boardcore::Canbus::CanbusDriver canPhysical, Boards source)
        : source(source)
    {
        *can = Boardcore::Canbus::CanProtocol(&canPhysical, &buffer);

        can->start();
    }

    void sendCan(Boards destination, Priority p, Type t, uint8_t idT,
                 uint64_t toSend[32], uint8_t nPacket)
    {
        if (nPacket > 0)
        {

            Boardcore::Canbus::CanData packet;
            packet.len   = nPacket;
            packet.canId = (p & idMask.priority) | (t & idMask.type) |
                           (source & idMask.source) |
                           (destination & idMask.destination) |
                           (idT & idMask.idType);
            memcpy(packet.payload, toSend, nPacket);
            can->sendCan(packet);
        }
    }

    /* Destructor */
    ~CanHandler() {}

protected:
    void run() override
    {
        Boardcore::Canbus::CanData data;

        while (true)
        {
            buffer.waitUntilNotEmpty();
            if (!buffer.isEmpty())
            {
                data = buffer.pop();

                switch (data.canId & idMask.type)
                {
                    case Events:
                        switch (data.canId & idMask.idType)
                        {
                            case Pitot:
                                /* code */
                                break;
                        }
                        /* code */
                        break;
                    case Command:
                        switch (data.canId & idMask.idType)
                        {
                            case Liftoff:
                                /* code */
                                break;
                            case Apogee:
                                /* code */
                                break;
                            case Armed:
                                /* code */
                                break;
                        }
                        break;

                    case Sensor:
                        switch (data.canId & idMask.idType)
                        {
                            case AirBrakes:
                                /* code */
                                break;
                        }
                        break;
                }
            }
        }
    }
};
}  // namespace common
