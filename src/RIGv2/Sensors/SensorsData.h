/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <sensors/ADS131M08/ADS131M08Data.h>
#include <sensors/MAX31856/MAX31856Data.h>
#include <sensors/SensorData.h>

namespace RIGv2
{

struct ADC1Data : Boardcore::ADS131M08Data
{
    explicit ADC1Data(const Boardcore::ADS131M08Data& data)
        : Boardcore::ADS131M08Data(data)
    {
    }

    ADC1Data() {}
};

struct TC1Data : Boardcore::MAX31856Data
{
    explicit TC1Data(const Boardcore::MAX31856Data& data)
        : Boardcore::MAX31856Data(data)
    {
    }

    TC1Data() {}
};

struct VesselWeightData : Boardcore::LoadCellData
{
    explicit VesselWeightData(const Boardcore::LoadCellData& data)
        : Boardcore::LoadCellData(data)
    {
    }

    VesselWeightData() {}
};

struct TankWeightData : Boardcore::LoadCellData
{
    explicit TankWeightData(const Boardcore::LoadCellData& data)
        : Boardcore::LoadCellData(data)
    {
    }

    TankWeightData() {}
};

struct VesselPressureData : Boardcore::PressureData
{
    explicit VesselPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    VesselPressureData() {}
};

struct FillingPressureData : Boardcore::PressureData
{
    explicit FillingPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    FillingPressureData() {}
};

struct TopTankPressureData : Boardcore::PressureData
{
    explicit TopTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    TopTankPressureData() {}
};

struct BottomTankPressureData : Boardcore::PressureData
{
    explicit BottomTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    BottomTankPressureData() {}
};

}  // namespace RIGv2
