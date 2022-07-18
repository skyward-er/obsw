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

#include "HIL_sensors/HILSensor.h"
#include "algorithms/AirBrakes/TrajectoryPoint.h"

struct HILKalmanData : public Boardcore::TimedTrajectoryPoint
{
    HILKalmanData() : TimedTrajectoryPoint() {}
};

class HILKalman : public HILSensor<HILKalmanData>
{
public:
    HILKalman(HILTransceiver *matlab, int n_data_sensor)
        : HILSensor(matlab, n_data_sensor)
    {
    }

protected:
    HILKalmanData updateData() override
    {
        /*
         * In here you put the kalman algorithm that from the sensorData
         * extracts the state
         */

        HILKalmanData tempData;
        tempData.z         = sensorData->kalman.z;
        tempData.vz        = sensorData->kalman.vz;
        tempData.vMod      = sensorData->kalman.vMod;
        tempData.timestamp = updateTimestamp();

        return tempData;
    }
};