/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#pragma once

#include <drivers/piksi/piksi_data.h>
#include <ostream>

namespace DeathStackBoard
{
struct PiksiData
{
    //GPSData gps_data;
    bool fix = false;

    static std::string header()
    {
        return "timestamp,lat,lon,alt,vel_n,vel_e,vel_d,speed,num_sat,fix\n";
    }

    void print(std::ostream& os) const
    {
        /*os << gps_data.timestamp << "," << gps_data.latitude << ","
           << gps_data.longitude << "," << gps_data.height << ","
           << gps_data.velocityNorth << "," << gps_data.velocityEast << ","
           << gps_data.velocityDown << "," << gps_data.speed << ","
           << gps_data.numSatellites << "," << (int)fix << "\n";*/
    }
};
}  // namespace DeathStackBoard