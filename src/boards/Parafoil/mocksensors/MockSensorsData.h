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

#include <sensors/SensorData.h>

struct MockIMUData : public Boardcore::AccelerometerData,
                     public Boardcore::GyroscopeData,
                     public Boardcore::MagnetometerData
{
    static std::string header()
    {
        return "accel_timestamp,accel_x,accel_y,accel_z,gyro_timestamp,gyro_x,"
               "gyro_y,"
               "gyro_z,mag_timestamp,mag_x,mag_y,mag_z\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << ","
           << angularVelocityTimestamp << "," << angularVelocityX << ","
           << angularVelocityY << "," << angularVelocityZ << ","
           << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
    }
};

struct MockPressureData : public PressureData
{
    static std::string header()
    {
        return "press_timestamp,press,temp_timestamp,temp\n";
    }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << "\n";
    }
};

struct MockGPSData : public GPSData
{
    static std::string header()
    {
        return "gps_timestamp,latitude,longitude,height,velocity_north,"
               "velocity_east,velocity_down,speed,track,num_satellites,fix\n";
    }

    void print(std::ostream& os) const
    {
        os << gpsTimestamp << "," << latitude << "," << longitude << ","
           << height << "," << velocityNorth << "," << velocityEast << ","
           << velocityDown << "," << speed << "," << track << ","
           << (int)satellites << "," << (int)fix << "\n";
    }
};

struct MockSpeedData
{
    uint64_t timestamp;
    float speed;

    static std::string header() { return "timestamp,speed\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << speed << "\n";
    }
};
