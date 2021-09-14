/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <configs/NASConfig.h>
#include <math/SkyQuaternion.h>
#include <stdint.h>

#include <iostream>
#include <string>

namespace DeathStackBoard
{

/**
 * @brief Enum defining the possibile FSM states.
 */
enum class NASState : uint8_t
{
    IDLE = 0,
    CALIBRATING,
    READY,
    ACTIVE,
    END,
};

/**
 * @brief Structure defining the overall NAS status.
 */
struct NASStatus
{
    uint64_t timestamp;
    NASState state;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

/**
 * @brief NavigationSystem output, used by the airbrakes algorithm.
 */
struct NASData
{
    uint64_t timestamp = 0;
    float x            = 0;
    float y            = 0;
    float z            = 0;
    float vx           = 0;
    float vy           = 0;
    float vz           = 0;
    float vMod         = 0;

    static std::string header() { return "timestamp,x,y,z,vx,vy,vz,vMod\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << x << "," << y << "," << z << "," << vx << ","
           << vy << "," << vz << "," << vMod << ","
           << "\n";
    }
};

/**
 * @brief Struct to log the Kalman states.
 */
struct NASKalmanState
{
    uint64_t timestamp = 0;
    float x0           = 0;
    float x1           = 0;
    float x2           = 0;
    float x3           = 0;
    float x4           = 0;
    float x5           = 0;
    float x6           = 0;
    float x7           = 0;
    float x8           = 0;
    float x9           = 0;
    float x10          = 0;
    float x11          = 0;
    float x12          = 0;

    NASKalmanState() {}

    NASKalmanState(uint64_t t, const Matrix<float, NASConfigs::N, 1>& state)
    {
        timestamp = t;
        x0        = state(0);
        x1        = state(1);
        x2        = state(2);
        x3        = state(3);
        x4        = state(4);
        x5        = state(5);
        x6        = state(6);
        x7        = state(7);
        x8        = state(8);
        x9        = state(9);
        x10       = state(10);
        x11       = state(11);
        x12       = state(12);
    }

    static std::string header()
    {
        return "timestamp,x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << x0 << "," << x1 << "," << x2 << "," << x3
           << "," << x4 << "," << x5 << "," << x6 << "," << x7 << "," << x8
           << "," << x9 << "," << x10 << "," << x11 << "," << x12 << "\n";
    }

    Vector3f toEul() const
    {
        SkyQuaternion q;
        return q.quat2eul({x6, x7, x8, x9});
    }
};

/**
 * @brief Struct to log reference values.
 *
 * Also used in NAS to store the values.
 */
struct NASReferenceValues
{
    float ref_pressure;

    float ref_temperature;

    float ref_altitude;
    float ref_latitude;
    float ref_longitude;

    float ref_accel_x;
    float ref_accel_y;
    float ref_accel_z;

    float ref_mag_x;
    float ref_mag_y;
    float ref_mag_z;

    static std::string header()
    {
        return "ref_pressure,ref_temperature,ref_altitude,ref_latitude,ref_"
               "longitude,ref_"
               "accel_x,ref_accel_y,ref_accel_z,ref_mag_x,ref_mag_y,ref_mag_"
               "z\n";
    }

    void print(std::ostream& os) const
    {
        os << ref_pressure << "," << ref_temperature << "," << ref_altitude
           << "," << ref_latitude << "," << ref_longitude << "," << ref_accel_x
           << "," << ref_accel_y << "," << ref_accel_z << "," << ref_mag_x
           << "," << ref_mag_y << "," << ref_mag_z << "\n";
    }

    bool operator==(const NASReferenceValues& other) const
    {
        return ref_pressure == other.ref_pressure &&
               ref_temperature == other.ref_temperature &&
               ref_altitude == other.ref_altitude &&
               ref_latitude == other.ref_latitude &&
               ref_longitude == other.ref_longitude &&
               ref_accel_x == other.ref_accel_x &&
               ref_accel_y == other.ref_accel_y &&
               ref_accel_z == other.ref_accel_z &&
               ref_mag_x == other.ref_mag_x && ref_mag_y == other.ref_mag_y &&
               ref_mag_z == other.ref_mag_z;
    }

    bool operator!=(const NASReferenceValues& other) const
    {
        return !(*this == other);
    }
};

struct NASTriadResult
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    static std::string header() { return "x,y,z,roll,pitch,yaw\n"; }

    void print(std::ostream& os) const
    {
        os << x << "," << y << "," << z << "," << roll << "," << pitch << ","
           << yaw << "\n";
    }
};

}  // namespace DeathStackBoard
