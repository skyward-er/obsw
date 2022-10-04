/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <logger/Logger.h>

#include "HILAccelerometer.h"
#include "HILGyroscope.h"
#include "HILMagnetometer.h"
#include "HILSensor.h"

/**
 * @brief fake 9-axis IMU sensor used for the simulation.
 *
 * This class is used to simulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a simulator.
 */
class HILImu : public HILSensor<HILImuData>
{
public:
    HILImu(int n_data_sensor) : HILSensor(n_data_sensor) {}

protected:
    HILImuData updateData() override
    {
        HILImuData tempData;
        Boardcore::Vec3 matlabData;

        /* I make a copy of the vector i have to memorize in the sensor
         * struct */
        matlabData = sensorData->accelerometer.measures[sampleCounter];
        tempData.accelerationX = matlabData.getX();
        tempData.accelerationY = matlabData.getY();
        tempData.accelerationZ = matlabData.getZ();

        matlabData                = sensorData->gyro.measures[sampleCounter];
        tempData.angularVelocityX = matlabData.getX();
        tempData.angularVelocityY = matlabData.getY();
        tempData.angularVelocityZ = matlabData.getZ();

        matlabData = sensorData->magnetometer.measures[sampleCounter];
        tempData.magneticFieldX = matlabData.getX();
        tempData.magneticFieldY = matlabData.getY();
        tempData.magneticFieldZ =
            matlabData.getZ() / 1000.0f;  // from nanotesla to microtesla

        // only update the timestamp once and use it for all the 3 sensors
        // (this sensor assumes the same frequency for accel, gyro and mag)
        tempData.accelerationTimestamp    = updateTimestamp();
        tempData.angularVelocityTimestamp = tempData.accelerationTimestamp;
        tempData.magneticFieldTimestamp   = tempData.accelerationTimestamp;

        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }
};
