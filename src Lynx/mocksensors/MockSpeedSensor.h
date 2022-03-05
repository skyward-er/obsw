/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <mocksensors/MockSensorsData.h>
#include <mocksensors/lynx_flight_data/lynx_airspeed_data.h>
#include <sensors/Sensor.h>

#include <random>

namespace DeathStackBoard
{

class MockSpeedSensor : public Boardcore::Sensor<MockSpeedData>
{
public:
    MockSpeedSensor() {}

    bool init() override { return true; }

    bool selfTest() override { return true; }

    MockSpeedData sampleImpl() override
    {
        MockSpeedData data;

        data.timestamp =
            Boardcore::TimestampTimer::getInstance().getTimestamp();

        if (before_liftoff)
        {
            data.speed = AIRSPEED_DATA[0];
        }
        else
        {
            if (i < AIRSPEED_DATA_SIZE)
            {
                data.speed = AIRSPEED_DATA[i++];
            }
            else
            {
                data.speed = AIRSPEED_DATA[AIRSPEED_DATA_SIZE - 1];
            }
        }

        return data;
    }

    void signalLiftoff() { before_liftoff = false; }

private:
    volatile bool before_liftoff = true;
    volatile unsigned int i      = 0;  // Last index
};

}  // namespace DeathStackBoard
