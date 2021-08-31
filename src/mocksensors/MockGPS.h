/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <tests/mock_sensors/test-mock-data.h>

#include "TimestampTimer.h"
#include "sensors/Sensor.h"

namespace DeathStackBoard
{

class MockGPS : public Sensor<GPSData>
{
public:
    MockGPS() {}

    bool init() override { return true; }

    bool selfTest() override { return true; }

    GPSData sampleImpl() override
    {
        // if (inside_lha)
        // {
        //     lat = lat_inside;
        //     lon = lon_inside;
        // }
        // else
        // {
        //     lat = lat_outside;
        //     lon = lon_outside;
        // }

        GPSData data;

        data.gps_timestamp = TimestampTimer::getTimestamp();
        data.fix           = true;

        if (before_liftoff)
        {
            data.latitude       = SIMULATED_LAT[0];
            data.longitude      = SIMULATED_LON[0];
            data.velocity_north = SIMULATED_VNORD[0];
            data.velocity_east  = SIMULATED_VEAST[0];
        }
        else if (i < GPS_DATA_SIZE)
        {
            data.latitude       = SIMULATED_LAT[i];
            data.longitude      = SIMULATED_LON[i];
            data.velocity_north = SIMULATED_VNORD[i];
            data.velocity_east  = SIMULATED_VEAST[i];
            i++;
        }

        return data;
    }

    void signalLiftoff() { before_liftoff = false; }

private:
    // Set of coordinates inside the Launch Hazard Area
    // const double lat_inside = 41.807487124105;
    // const double lon_inside = 14.0551665469291;

    // Set of coordinates outside the Launch Hazard Area
    // const double lat_outside = 41.840794;
    // const double lon_outside = 14.003920;

    volatile bool before_liftoff = true;
    bool inside_lha              = true;

    unsigned int i = 0;
};
}  // namespace DeathStackBoard
