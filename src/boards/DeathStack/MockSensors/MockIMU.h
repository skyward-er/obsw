/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Marco Cella
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

#pragma once

#include <tests/mock_sensors/test-mock-data.h>

#include "TimestampTimer.h"
#include "sensors/Sensor.h"

namespace DeathStackBoard
{

struct MockIMUData : public AccelerometerData,
                     public GyroscopeData,
                     public MagnetometerData
{
};

class MockIMU : public Sensor<MockIMUData>
{
public:
    MockIMU() {}

    bool init() override { return true; }

    bool selfTest() override { return true; }

    MockIMUData sampleImpl() override
    {
        if (before_liftoff)
        {
            index = 0;
        }

        MockIMUData data;
        
        data.accel_timestamp = TimestampTimer::getTimestamp();
        data.accel_x         = ACCELEROMETER_DATA[0][index];
        data.accel_y         = ACCELEROMETER_DATA[1][index];
        data.accel_z         = ACCELEROMETER_DATA[2][index];

        data.gyro_timestamp = TimestampTimer::getTimestamp();
        data.gyro_x         = GYROSCOPE_DATA[0][index];
        data.gyro_y         = GYROSCOPE_DATA[1][index];
        data.gyro_z         = GYROSCOPE_DATA[2][index];

        data.mag_timestamp = TimestampTimer::getTimestamp();
        data.mag_x         = MAGNETOMETER_DATA[0][index];
        data.mag_y         = MAGNETOMETER_DATA[1][index];
        data.mag_z         = MAGNETOMETER_DATA[2][index];

        // when finished, go back to the beginning
        index = (index + 1) % IMU_DATA_SIZE;

        return data;
    }

    void signalLiftoff() { before_liftoff = false; }

private:
    unsigned int index           = 0;
    volatile bool before_liftoff = true;
};

}  // namespace DeathStackBoard
