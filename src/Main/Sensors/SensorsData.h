/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

namespace Main
{

struct StaticPressureData1 : Boardcore::PressureData
{
    explicit StaticPressureData1(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressureData1() {}
};

struct StaticPressureData2 : Boardcore::PressureData
{
    explicit StaticPressureData2(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressureData2() {}
};

struct DplBayPressureData : Boardcore::PressureData
{
    explicit DplBayPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    DplBayPressureData() {}
};

struct CalibrationData
{
    uint64_t timestamp      = 0;
    float gyroBiasX         = 0.0f;
    float gyroBiasY         = 0.0f;
    float gyroBiasZ         = 0.0f;
    float magBiasX          = 0.0f;
    float magBiasY          = 0.0f;
    float magBiasZ          = 0.0f;
    float magScaleX         = 0.0f;
    float magScaleY         = 0.0f;
    float magScaleZ         = 0.0f;
    float staticPress1Bias  = 0.0f;
    float staticPress1Scale = 0.0f;
    float staticPress2Bias  = 0.0f;
    float staticPress2Scale = 0.0f;
    float dplBayPressBias   = 0.0f;
    float dplBayPressScale  = 0.0f;

    static std::string header()
    {
        return "timestamp,gyroBiasX,gyroBiasY,gyroBiasZ,magBiasX,magBiasY,"
               "magBiasZ,magScaleX,magScaleY,magScaleZ,staticPress1Bias,"
               "staticPress1Scale,staticPress2Bias,staticPress2Scale,"
               "dplBayPressBias,dplBayPressScale\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << gyroBiasX << "," << gyroBiasY << ","
           << gyroBiasZ << "," << magBiasX << "," << magBiasY << "," << magBiasZ
           << "," << magScaleX << "," << magScaleY << "," << magScaleZ << ","
           << staticPress1Bias << "," << staticPress1Scale << ","
           << staticPress2Bias << "," << staticPress2Scale << ","
           << dplBayPressBias << "," << dplBayPressScale << "\n";
    }
};

}  // namespace Main
