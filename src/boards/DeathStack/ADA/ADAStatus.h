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

#include <boards/DeathStack/configs/ADA_config.h>
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
// Also used to keep track of current state
struct ADAControllerStatus
{
    long long timestamp;
    ADAState state            = ADAState::UNDEFINED;
    bool apogee_reached       = false;
    bool dpl_altitude_reached = false;

    static std::string header()
    {
        return "timestamp,state,apogee_reached,dpl_altitude_"
               "reached\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "," << apogee_reached << ","
           << dpl_altitude_reached << "\n";
    }
};

// Struct to log the two Kalman states
struct KalmanState
{
    long long timestamp;
    float x0;
    float x1;
    float x2;
    float x0_acc;
    float x1_acc;
    float x2_acc;

    static std::string header()
    {
        return "timestamp,x0,x1,x2,x0_acc,x1_acc,x2_acc\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << x0 << "," << x1 << "," << x2 << "," << x0_acc
           << "," << x1_acc << "," << x2_acc << "\n";
    }
};

// Struct to log altitude and vertical speed of first Kalman
// (state after conversion)
struct ADAData
{
    long long timestamp;
    float msl_altitude;
    float dpl_altitude;
    bool is_dpl_altitude_agl;
    float vert_speed;
    
    float acc_msl_altitude;
    float acc_vert_speed;
    
    static std::string header()
    {
        return "timestamp,msl_altitude,dpl_altitude,is_agl,vert_speed,acc_msl_altitude,acc_vert_speed\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << msl_altitude << "," << dpl_altitude
           << "," << (int)is_dpl_altitude_agl << "," << vert_speed  << "," << acc_msl_altitude  << "," << acc_vert_speed << "\n";
    }
};

// Struct to log altimeter reference values
// Also used in ADA to store the values
struct ReferenceValues
{
    // Launch site altitude
    float ref_altitude = DEFAULT_REFERENCE_ALTITUDE;

    // Launch site pressure and temperature
    float ref_pressure    = DEFAULT_REFERENCE_PRESSURE;
    float ref_temperature = DEFAULT_REFERENCE_TEMPERATURE;

    // Pressure at mean sea level for altitude calculation, to be updated with
    // launch-day calibration
    float msl_pressure    = DEFAULT_MSL_PRESSURE;
    float msl_temperature = DEFAULT_MSL_TEMPERATURE;

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

// Struct to log dpl altitude
struct TargetDeploymentAltitude
{
    float deployment_altitude;

    static std::string header() { return "deployment_altitude\n"; }

    void print(std::ostream& os) const { os << deployment_altitude << "\n"; }
};

}  // namespace DeathStackBoard