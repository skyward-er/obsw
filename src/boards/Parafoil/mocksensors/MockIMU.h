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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <TimestampTimer.h>
#include <sensors/Sensor.h>
#include <mocksensors/MockSensorsData.h>
#include <mocksensors/lynx_flight_data/lynx_imu_data.h>

namespace DeathStackBoard
{

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
        uint64_t t = TimestampTimer::getTimestamp();

        data.accel_timestamp = t;
        data.accel_x         = ACCEL_DATA[index][0];
        data.accel_y         = ACCEL_DATA[index][1];
        data.accel_z         = ACCEL_DATA[index][2];

        data.gyro_timestamp = t;
        data.gyro_x         = GYRO_DATA[index][0];
        data.gyro_y         = GYRO_DATA[index][1];
        data.gyro_z         = GYRO_DATA[index][2];

        data.mag_timestamp = t;
        data.mag_x         = MAG_DATA[index][0];
        data.mag_y         = MAG_DATA[index][1];
        data.mag_z         = MAG_DATA[index][2];

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
