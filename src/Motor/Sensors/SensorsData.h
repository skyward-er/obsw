/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Fabrizio Monti, Niccolò Betto, RIccardo Sironi
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

#include <reflect.hpp>

namespace Motor
{

struct RegulatorOutOxPressureData : Boardcore::PressureData
{
    explicit RegulatorOutOxPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    RegulatorOutOxPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(RegulatorOutOxPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct RegulatorOutFuelPressureData : Boardcore::PressureData
{
    explicit RegulatorOutFuelPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    RegulatorOutFuelPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(RegulatorOutFuelPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxTankPressureData : Boardcore::PressureData
{
    explicit OxTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    OxTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct PrzTankPressureData : Boardcore::PressureData
{
    explicit PrzTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    PrzTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct FuelTankPressureData : Boardcore::PressureData
{
    explicit FuelTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    FuelTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FuelTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct CCPressureData : Boardcore::PressureData
{
    explicit CCPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    CCPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CCPressureData, EXTEND_DEF(Boardcore::PressureData));
    }
};

struct IgniterPressureData : Boardcore::PressureData
{
    explicit IgniterPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    IgniterPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(IgniterPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct MainFuelPositionData : Boardcore::ServoPositionData
{
    explicit MainFuelPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData{data}
    {
    }

    MainFuelPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MainFuelPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct PrzOxPositionData : Boardcore::ServoPositionData
{
    explicit PrzOxPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData{data}
    {
    }

    PrzOxPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzOxPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct PrzFuelPositionData : Boardcore::ServoPositionData
{
    explicit PrzFuelPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData{data}
    {
    }

    PrzFuelPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzFuelPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct VentingOxPositionData : Boardcore::ServoPositionData
{
    explicit VentingOxPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData{data}
    {
    }

    VentingOxPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(VentingOxPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct VentingFuelPositionData : Boardcore::ServoPositionData
{
    explicit VentingFuelPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData{data}
    {
    }

    VentingFuelPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(VentingFuelPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

struct PrzOxPositionData : Boardcore::ServoPositionData
{
    explicit PrzOxPositionData(const Boardcore::ServoPositionData& data)
        : Boardcore::ServoPositionData{data}
    {
    }

    PrzOxPositionData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PrzOxPositionData,
                          EXTEND_DEF(Boardcore::ServoPositionData));
    }
};

}  // namespace Motor
