/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Mozzarelli
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

#include <configs/ADAConfig.h>
#include <configs/config.h>
#include <math/Stats.h>
#include <utils/Constants.h>

#include <ostream>

#include <ostream>

namespace DeathStackBoard
{

/**
 * @brief All possible states of the ADA.
 */
enum class ADAState : uint8_t
{
    IDLE = 0,
    CALIBRATING,
    READY,
    SHADOW_MODE,
    ACTIVE,
    PRESSURE_STABILIZATION,
    DROGUE_DESCENT,
    END
};

/**
 * @brief Struct to log altitude and vertical speed of first Kalman (state after
 * conversion).
 */
struct ADAData
{
    uint64_t timestamp;
    float msl_altitude;
    float agl_altitude;
    float vert_speed;

    static std::string header()
    {
        return "timestamp,msl_altitude,agl_altitude,vert_speed\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << msl_altitude << "," << agl_altitude << ","
           << vert_speed << "\n";
    }
};

/**
 * @brief Struct to log current state, also used to keep track of current state.
 */
struct ADAControllerStatus
{
    uint64_t timestamp;
    ADAState state            = ADAState::IDLE;
    bool apogee_reached       = false;
    bool disable_airbrakes    = false;
    bool dpl_altitude_reached = false;

    static std::string header()
    {
        return "timestamp,state,apogee_reached,disable_airbrakes,dpl_altitude_"
               "reached\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "," << apogee_reached << ","
           << disable_airbrakes << "," << dpl_altitude_reached << "\n";
    }
};

/**
 * @brief Struct to log apogee detection.
 */
struct ApogeeDetected
{
    uint64_t timestamp;
    ADAState state;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

/**
 * @brief Struct to log deployment pressure detection.
 */
struct DplAltitudeReached
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

/**
 * @brief Struct to log the two Kalman states.
 */
struct ADAKalmanState
{
    uint64_t timestamp;
    float x0;
    float x1;
    float x2;

    static std::string header() { return "timestamp,x0,x1,x2\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << x0 << "," << x1 << "," << x2 << "\n";
    }
};

/**
 * @brief Struct to log altimeter reference values, also used in ADA to store
 * the values.
 */
struct ADAReferenceValues
{
    // Launch site altitude
    float ref_altitude = DEFAULT_REFERENCE_ALTITUDE;

    // Launch site pressure and temperature
    float ref_pressure    = DEFAULT_REFERENCE_PRESSURE;
    float ref_temperature = DEFAULT_REFERENCE_TEMPERATURE;

    // Pressure at mean sea level for altitude calculation, to be updated with
    // launch-day calibration
    float msl_pressure    = Boardcore::MSL_PRESSURE;
    float msl_temperature = Boardcore::MSL_TEMPERATURE;

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

    bool operator==(const ADAReferenceValues& other) const
    {
        return ref_altitude == other.ref_altitude &&
               ref_pressure == other.ref_pressure &&
               ref_temperature == other.ref_temperature &&
               msl_pressure == other.msl_pressure &&
               msl_temperature == other.msl_temperature;
    }

    bool operator!=(const ADAReferenceValues& other) const
    {
        return !(*this == other);
    }
};

/**
 * @brief Struct to log dpl altitude.
 */
struct TargetDeploymentAltitude
{
    float deployment_altitude = DEFAULT_DEPLOYMENT_ALTITUDE;

    static std::string header() { return "deployment_altitude\n"; }

    void print(std::ostream& os) const { os << deployment_altitude << "\n"; }
};

}  // namespace DeathStackBoard
