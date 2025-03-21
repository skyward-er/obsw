/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Pietro Bortolus
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

#include <sensors/LIS2MDL/LIS2MDLData.h>
#include <sensors/LSM6DSRX/LSM6DSRXData.h>
#include <sensors/SensorData.h>

namespace Main
{

struct StaticPressure0Data : public Boardcore::PressureData
{
    explicit StaticPressure0Data(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressure0Data() {}
};

struct StaticPressure1Data : Boardcore::PressureData
{
    explicit StaticPressure1Data(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressure1Data() {}
};

struct StaticPressure2Data : Boardcore::PressureData
{
    explicit StaticPressure2Data(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressure2Data() {}
};

struct DplBayPressureData : Boardcore::PressureData
{
    explicit DplBayPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    DplBayPressureData() {}
};

struct LSM6DSRX0Data : Boardcore::LSM6DSRXData
{
    explicit LSM6DSRX0Data(const Boardcore::LSM6DSRXData& data)
        : Boardcore::LSM6DSRXData(data)
    {
    }

    LSM6DSRX0Data() {}
};

struct LSM6DSRX1Data : Boardcore::LSM6DSRXData
{
    explicit LSM6DSRX1Data(const Boardcore::LSM6DSRXData& data)
        : Boardcore::LSM6DSRXData(data)
    {
    }

    LSM6DSRX1Data() {}
};

struct LIS2MDLExternalData : Boardcore::LIS2MDLData
{
    explicit LIS2MDLExternalData(const Boardcore::LIS2MDLData& data)
        : Boardcore::LIS2MDLData(data)
    {
    }

    LIS2MDLExternalData() {}
};

struct CalibrationData
{
    uint64_t timestamp = 0;
    float gyroBiasX    = 0.0f;
    float gyroBiasY    = 0.0f;
    float gyroBiasZ    = 0.0f;
    float magBiasX     = 0.0f;
    float magBiasY     = 0.0f;
    float magBiasZ     = 0.0f;
    float magScaleX    = 0.0f;
    float magScaleY    = 0.0f;
    float magScaleZ    = 0.0f;

    static std::string header()
    {
        return "timestamp,gyroBiasX,gyroBiasY,gyroBiasZ,magBiasX,magBiasY,"
               "magBiasZ,magScaleX,magScaleY,magScaleZ\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << gyroBiasX << "," << gyroBiasY << ","
           << gyroBiasZ << "," << magBiasX << "," << magBiasY << "," << magBiasZ
           << "," << magScaleX << "," << magScaleY << "," << magScaleZ << "\n";
    }
};

}  // namespace Main
