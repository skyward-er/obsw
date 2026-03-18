/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Pietro Bortolus
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
#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace RIGv3
{

struct ADC0Data : Boardcore::ADS131M08Data
{
    explicit ADC0Data(const Boardcore::ADS131M08Data& data)
        : Boardcore::ADS131M08Data(data)
    {
    }

    ADC0Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADC0Data, EXTEND_DEF(Boardcore::ADS131M08Data));
    }
};

struct PrzVessel1PressureData : Boardcore::PressureData
{
    explicit PrzVessel1PressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    PrzVessel1PressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzVessel1PressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct PrzVessel2PressureData : Boardcore::PressureData
{
    explicit PrzVessel2PressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    PrzVessel2PressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzVessel2PressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct PrzFillingPressureData : Boardcore::PressureData
{
    explicit PrzFillingPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    PrzFillingPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzFillingPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
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

struct PrzTankPressureData : Boardcore::PressureData
{
    explicit PrzTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    PrzTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxRegOutPressureData : Boardcore::PressureData
{
    explicit OxRegOutPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    OxRegOutPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxRegOutPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct FuelRegOutPressureData : Boardcore::PressureData
{
    explicit FuelRegOutPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    FuelRegOutPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FuelRegOutPressureData,
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

struct FuelTankPressureData : Boardcore::PressureData
{
    explicit FuelTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    FuelTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FuelTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct IgniterChamberPressureData : Boardcore::PressureData
{
    explicit IgniterChamberPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    IgniterChamberPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(IgniterChamberPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct MainChamberPressureData : Boardcore::PressureData
{
    explicit MainChamberPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    MainChamberPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MainChamberPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct ADC3Data : Boardcore::ADS131M08Data
{
    explicit ADC3Data(const Boardcore::ADS131M08Data& data)
        : Boardcore::ADS131M08Data(data)
    {
    }

    ADC3Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADC3Data, EXTEND_DEF(Boardcore::ADS131M08Data));
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

struct MainOxPositionData : Boardcore::ServoPositionData
{
    explicit MainOxPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData(data)
    {
    }

    MainOxPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MainOxPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct MainFuelPositionData : Boardcore::ServoPositionData
{
    explicit MainFuelPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData(data)
    {
    }

    MainFuelPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MainFuelPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct OxRegPositionData : Boardcore::ServoPositionData
{
    explicit OxRegPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData(data)
    {
    }

    OxRegPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxRegPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct FuelRegPositionData : Boardcore::ServoPositionData
{
    explicit FuelRegPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData(data)
    {
    }

    FuelRegPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FuelRegPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

}  // namespace RIGv3
