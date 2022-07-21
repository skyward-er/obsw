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

#include <events/EventBroker.h>
namespace common
{
uint32_t BAUD_RATE = 500 * 1000;
float SAMPLE_POINT = 87.5f / 100.0f;

enum CanEvent : uint8_t
{
    EV_LIFTOFF = Boardcore::EV_FIRST_CUSTOM,
    EV_APOGEE,
    EV_ARMED,
    EV_AIRBRAKES
};

enum CanTopics : uint8_t
{
    TOPIC_CAN_EVENTS
};

enum SensorID : uint8_t
{
    AirBrakes = 0x00,
    Pitot     = 0x01,
    NumberOfSensor
};

enum EventsId : uint8_t
{
    Liftoff = 0x00,
    Apogee  = 0x01,
    Armed   = 0x02
};

enum Boards : uint8_t
{
    Broadcast = 0x00,
    Main      = 0x01,
    Payload   = 0x02,
    Auxiliary = 0x03
};

enum Priority : uint8_t
{
    Critical = 0x00,
    High     = 0x01,
    Medium   = 0x02,
    Low      = 0x03
};

enum Type : uint8_t
{
    Events = 0x00,
    Sensor = 0x01

};
}  // namespace common
