/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_TESTSENSOR_H
#define SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_TESTSENSOR_H

#include <cmath>

#include "Common.h"
#include "sensors/Sensor.h"

using miosix::getTick;
using miosix::TICK_FREQ;

struct TestSensorData : public TimestampData
{
    float value;

    TestSensorData()
        : TimestampData{0}, value(0.0)
    {
    }

    TestSensorData(uint64_t timestamp, float value)
        : TimestampData{timestamp}, value(value)
    {
    }
};

class TestSensor : public Sensor<TestSensorData>
{
public:
    TestSensor() {}
    virtual ~TestSensor() {}

    bool init() { return true; }

    bool selfTest() { return true; }

    TestSensorData sampleImpl()
    {
        float v = 10 * sin(PI * static_cast<float>(getTick()) /
                               static_cast<float>(TICK_FREQ));

        return TestSensorData(static_cast<uint64_t>(getTick()), v);
    }
};

#endif /* SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_TESTSENSOR_H */
