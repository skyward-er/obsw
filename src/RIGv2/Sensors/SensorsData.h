/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <reflect.hpp>

namespace RIGv2
{

struct ADC1Data : Boardcore::ADS131M08Data
{
    explicit ADC1Data(const Boardcore::ADS131M08Data& data)
        : Boardcore::ADS131M08Data(data)
    {
    }

    ADC1Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADC1Data, EXTEND_DEF(Boardcore::ADS131M08Data));
    }
};

struct ADC2Data : Boardcore::ADS131M08Data
{
    explicit ADC2Data(const Boardcore::ADS131M08Data& data)
        : Boardcore::ADS131M08Data(data)
    {
    }

    ADC2Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADC2Data, EXTEND_DEF(Boardcore::ADS131M08Data));
    }
};

struct TC1Data : Boardcore::MAX31856Data
{
    explicit TC1Data(const Boardcore::MAX31856Data& data)
        : Boardcore::MAX31856Data(data)
    {
    }

    TC1Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TC1Data, EXTEND_DEF(Boardcore::MAX31856Data));
    }
};

struct OxVesselWeightData : Boardcore::LoadCellData
{
    explicit OxVesselWeightData(const Boardcore::LoadCellData& data)
        : Boardcore::LoadCellData(data)
    {
    }

    OxVesselWeightData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxVesselWeightData,
                          EXTEND_DEF(Boardcore::LoadCellData));
    }
};

struct RocketWeightData : Boardcore::LoadCellData
{
    explicit RocketWeightData(const Boardcore::LoadCellData& data)
        : Boardcore::LoadCellData(data)
    {
    }

    RocketWeightData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(RocketWeightData,
                          EXTEND_DEF(Boardcore::LoadCellData));
    }
};

struct OxTankWeightData : Boardcore::LoadCellData
{
    explicit OxTankWeightData(const Boardcore::LoadCellData& data)
        : Boardcore::LoadCellData(data)
    {
    }

    OxTankWeightData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxTankWeightData,
                          EXTEND_DEF(Boardcore::LoadCellData));
    }
};

struct OxVesselPressureData : Boardcore::PressureData
{
    explicit OxVesselPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    OxVesselPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxVesselPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxFillingPressureData : Boardcore::PressureData
{
    explicit OxFillingPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    OxFillingPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxFillingPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct N2Vessel1PressureData : Boardcore::PressureData
{
    explicit N2Vessel1PressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    N2Vessel1PressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(N2Vessel1PressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct N2Vessel2PressureData : Boardcore::PressureData
{
    explicit N2Vessel2PressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    N2Vessel2PressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(N2Vessel2PressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct N2FillingPressureData : Boardcore::PressureData
{
    explicit N2FillingPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    N2FillingPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(N2FillingPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxTankPressureData : Boardcore::PressureData
{
    explicit OxTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    OxTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct N2TankPressureData : Boardcore::PressureData
{
    explicit N2TankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    N2TankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(N2TankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

}  // namespace RIGv2
