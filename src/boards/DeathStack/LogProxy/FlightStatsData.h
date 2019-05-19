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

#include <ostream>
#include <string>

namespace DeathStackBoard
{
struct LiftOffStats
{
    uint32_t T_liftoff = 0;

    uint32_t T_max_acc = 0;
    float acc_max      = 0.0f;

    uint32_t T_max_speed     = 0;
    float vert_speed_max     = 0.0f;
    float altitude_max_speed = 0.0f;

    static std::string header()
    {
        return "T_liftoff,T_max_acc,acc_max,T_max_speed,vert_speed_max,"
               "altitude_max_speed\n";
    }

    void print(std::ostream& os) const
    {
        os << T_liftoff << "," << T_max_acc << "," << acc_max << ","
           << T_max_speed << "," << vert_speed_max << "," << altitude_max_speed
           << "\n";
    }
};

struct ApogeeStats
{
    uint32_t T_apogee          = 0;
    float nxp_min_pressure     = 0.0f;
    float hw_min_pressure      = 0.0f;
    float kalman_min_pressure  = 0.0f;
    float digital_min_pressure = 0.0f;

    float baro_max_altitude = 0.0f;
    float gps_max_altitude  = 0.0f;

    float lat_apogee = 0.0f;
    float lon_apogee = 0.0f;

    static std::string header()
    {
        return "T_apogee,nxp_min_pressure,hw_min_pressure,kalman_min_pressure,"
               "digital_min_pressure,baro_max_altitude,gps_max_altitude,lat_"
               "apogee,lon_apogee\n";
    }

    void print(std::ostream& os) const
    {
        os << T_apogee << "," << nxp_min_pressure << "," << hw_min_pressure
           << "," << kalman_min_pressure << "," << digital_min_pressure << ","
           << baro_max_altitude << "," << gps_max_altitude << "," << lat_apogee
           << "," << lon_apogee << "\n";
    }
};

struct DrogueDPLStats
{
    uint32_t T_dpl    = 0;
    float max_dpl_acc = 0.0f;

    static std::string header() { return "T_dpl,max_dpl_acc\n"; }

    void print(std::ostream& os) const
    {
        os << T_dpl << "," << max_dpl_acc << "\n";
    }
};

struct MainDPLStats
{
    uint32_t T_dpl       = 0;
    float T_dpl          = 0.0f;
    float T_dpl          = 0.0f;
    float vert_speed_dpl = 0.0f;

    static std::string header() { return "T_dpl,T_dpl,T_dpl,vert_speed_dpl\n"; }

    void print(std::ostream& os) const
    {
        os << T_dpl << "," << T_dpl << "," << T_dpl << "," << vert_speed_dpl
           << "\n";
    }
};
}  // namespace DeathStackBoard