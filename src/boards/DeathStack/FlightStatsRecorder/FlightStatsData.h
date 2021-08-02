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
    uint64_t T_liftoff = 0;

    uint64_t T_max_acc = 0;
    float acc_max      = 0.0f;

    uint64_t T_max_speed     = 0;
    float vert_speed_max     = 0.0f;
    float airspeed_pitot_max = 0.0f;
    float altitude_max_speed = 0.0f;

    static std::string header()
    {
        return "T_liftoff,T_max_acc,acc_max,T_max_speed,vert_speed_max,"
               "airspeed_pitot_max,altitude_max_speed\n";
    }

    void print(std::ostream& os) const
    {
        os << T_liftoff << "," << T_max_acc << "," << acc_max << ","
           << T_max_speed << "," << vert_speed_max << "," << airspeed_pitot_max
           << "," << altitude_max_speed << "\n";
    }
};

struct CutterTestStats
{
    uint64_t timestamp = 0;
    float cutter_1_avg = 0;
    float cutter_2_avg = 0;
    uint32_t n_samples_1 = 0;
    uint32_t n_samples_2 = 0;
    static std::string header()
    {
        return "timestamp,cutter_1_avg,cutter_2_avg\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << cutter_1_avg << "," << cutter_2_avg << "\n";
    }
};

struct ApogeeStats
{
    uint64_t T_apogee          = 0;
    float static_min_pressure  = 200000.0f;
    float ada_min_pressure     = 200000.0f;
    float digital_min_pressure = 200000.0f;

    float baro_max_altitude = 0.0f;
    float gps_max_altitude  = 0.0f;

    float lat_apogee = 0.0f;
    float lon_apogee = 0.0f;

    static std::string header()
    {
        return "T_apogee,static_min_pressure,ada_min_pressure,digital_min_"
               "pressure,baro_max_altitude,gps_max_altitude,lat_apogee,lon_"
               "apogee\n";
    }

    void print(std::ostream& os) const
    {
        os << T_apogee << "," << static_min_pressure << "," << ada_min_pressure
           << "," << digital_min_pressure << "," << baro_max_altitude << ","
           << gps_max_altitude << "," << lat_apogee << "," << lon_apogee
           << "\n";
    }
};

struct DrogueDPLStats
{
    uint64_t T_dpl    = 0;
    float max_dpl_acc = 0.0f;
    float max_dpl_vane_pressure = 0.0f;

    static std::string header() { return "T_dpl,max_dpl_acc,max_dpl_vane_pressure\n"; }

    void print(std::ostream& os) const
    {
        os << T_dpl << "," << max_dpl_acc << "," << max_dpl_vane_pressure << "\n";
    }
};

struct MainDPLStats
{
    uint64_t T_dpl       = 0;
    float max_dpl_acc    = 0.0f;
    float altitude_dpl   = 0.0f;
    float vert_speed_dpl = 0.0f;

    static std::string header()
    {
        return "T_dpl,max_dpl_acc,altitude_dpl,vert_speed_dpl\n";
    }

    void print(std::ostream& os) const
    {
        os << T_dpl << "," << max_dpl_acc << "," << altitude_dpl << ","
           << vert_speed_dpl << "\n";
    }
};
}  // namespace DeathStackBoard