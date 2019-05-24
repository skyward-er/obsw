/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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

#include <math/Stats.h>
#include <ostream>

namespace DeathStackBoard
{

// All possible states of the ADA FMM
enum class ADAState
{
    UNDEFINED,
    IDLE,
    CALIBRATING,
    READY,
    SHADOW_MODE,
    ACTIVE,
    FIRST_DESCENT_PHASE,
    END
};

// Struct to log apogee detection
struct ApogeeDetected
{
    ADAState state;
    long long tick;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << tick << "," << (int)state << "\n";
    }
};

// Struct to log deployment pressure detection
struct DplAltitudeReached
{
    long long tick;
    static std::string header() { return "tick\n"; }

    void print(std::ostream& os) const { os << tick << "\n"; }
};

// Struct to log current state
struct ADAStatus
{
    long long timestamp;
    ADAState state            = ADAState::UNDEFINED;
    bool dpl_altitude_set     = false;
    bool ref_altitude_set     = false;
    bool ref_temp_set         = false;
    bool apogee_reached       = false;
    bool dpl_altitude_reached = false;

    static std::string header()
    {
        return "timestamp,state,dpl_altitude_set,apogee_reached,dpl_altitude_"
               "reached\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "," << dpl_altitude_set << ","
           << apogee_reached << "," << dpl_altitude_reached << "\n";
    }
};

struct KalmanState
{
    float x0;
    float x1;
    float x2;

    static std::string header() { return "x0,x1,x2\n"; }

    void print(std::ostream& os) const
    {
        os << x0 << "," << x1 << "," << x2 << "\n";
    }
};

struct KalmanAltitude
{
    long long timestamp;
    float altitude;
    float vert_speed;

    static std::string header() { return "timestamp,altitude,vert_speed\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << altitude << "," << vert_speed << "\n";
    }
};

struct ReferenceValues
{
    float ref_altitude = 0;

    float ref_pressure = 0;
    float ref_temperature = 0;

    float msl_pressure = 0;
    float msl_temperature = 0;

    static std::string header()
    {
        return "ref_altitude,ref_pressure,ref_temperature,msl_pressure,msl_"
               "temperature\n";
    }

    void print(std::ostream& os) const
    {
        os << ref_altitude << "," << ref_pressure << "," << ref_temperature
           << "," << msl_pressure << "," << msl_temperature << "\n";
    }
};

struct TargetDeploymentAltitude
{
    float deployment_altitude;
    
    static std::string header() { return "deployment_altitude\n"; }

    void print(std::ostream& os) const { os << deployment_altitude << "\n"; }
};

// Struct of calibration data
struct ADACalibrationData
{
    StatsResult pressure_calib;

    static std::string header()
    {
        return "pressure_calib.minValue,pressure_calib.maxValue,pressure_calib."
               "mean,pressure_calib.stdev,pressure_calib.nSamples\n";
    }

    void print(std::ostream& os) const
    {
        os << pressure_calib.minValue << "," << pressure_calib.maxValue << ","
           << pressure_calib.mean << "," << pressure_calib.stdev << ","
           << pressure_calib.nSamples << "\n";
    }
};

}  // namespace DeathStackBoard