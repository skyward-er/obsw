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

#include "HILSensor.h"
#include "math/Vec3.h"

/**
 * @brief fake gps sensor used for the HILulation.
 *
 * This class is used to HILulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a HILulator.
 */
class HILGps : public HILSensor<HILGpsData>
{
public:
    HILGps(HILTransceiver *matlab, int n_data_sensor)
        : HILSensor(matlab, n_data_sensor)
    {
    }

protected:
    HILGpsData updateData() override
    {
        HILGpsData tempData;

        /* I make a copy of the vector i have to memorize in the sensor
         * struct */
        Vec3 matlabData = sensorData->gps.positionMeasures[sampleCounter];
        tempData.latitude =
            matlabData.getX(); // divide by earth radius
        tempData.longitude = matlabData.getY();
        tempData.height    = matlabData.getZ();

        matlabData = sensorData->gps.velocityMeasures[sampleCounter];
        tempData.velocity_north = matlabData.getX();
        tempData.velocity_east  = matlabData.getY();
        tempData.velocity_down  = matlabData.getZ();

        tempData.fix            = (bool)sensorData->gps.fix;
        tempData.num_satellites = (uint8_t)sensorData->gps.num_satellites;

        tempData.gps_timestamp = updateTimestamp();

        return tempData;
    }
};