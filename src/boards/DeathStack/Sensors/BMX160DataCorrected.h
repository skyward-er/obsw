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

#include "sensors/BMX160/BMX160Data.h"

struct BMX160DataCorrected : public BMX160Data
{
    BMX160DataCorrected() : BMX160Data() {}

    BMX160DataCorrected(const BMX160Data& data)
        : BMX160DataCorrected(data, data, data)
    {
    }

    BMX160DataCorrected(AccelerometerData acc, GyroscopeData gyr,
                        MagnetometerData mag)
        : BMX160Data(acc, gyr, mag)
    {
    }

    static std::string header()
    {
        return "accel_timestamp,accel_x,accel_y,accel_z,gyro_timestamp,gyro_x,"
               "gyro_y,"
               "gyro_z,mag_timestamp,mag_x,mag_y,mag_z\n";
    }

    void print(std::ostream& os) const
    {
        os << accel_timestamp << "," << accel_x << "," << accel_y << ","
           << accel_z << "," << gyro_timestamp << "," << gyro_x << "," << gyro_y
           << "," << gyro_z << "," << mag_timestamp << "," << mag_x << ","
           << mag_y << "," << mag_z << "\n";
    }
};

struct BMX160GyroOffsets : public GyroscopeData
{
    BMX160GyroOffsets() : GyroscopeData{} {}

    BMX160GyroOffsets(GyroscopeData gyr)
        : GyroscopeData{gyr.gyro_timestamp, gyr.gyro_x, gyr.gyro_y, gyr.gyro_z}
    {
    }

    BMX160GyroOffsets(uint64_t t, float x, float y, float z)
        : GyroscopeData{t, x, y, z}
    {
    }

    static std::string header()
    {
        return "gyro_timestamp,gyro_x_offset,gyro_y_offset,"
               "gyro_z_offset\n";
    }

    void print(std::ostream& os) const
    {
        os << gyro_timestamp << "," << gyro_x << "," << gyro_y << "," << gyro_z
           << "\n";
    }
};