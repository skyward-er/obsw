/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Fabrizio Monti, Niccolò Betto
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

struct RegulatorOutPressureData : Boardcore::PressureData
{
    explicit RegulatorOutPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    RegulatorOutPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(RegulatorOutPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxTankTopPressureData : Boardcore::PressureData
{
    explicit OxTankTopPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    OxTankTopPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxTankTopPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxTankBottom0PressureData : Boardcore::PressureData
{
    explicit OxTankBottom0PressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    OxTankBottom0PressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxTankBottom0PressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct OxTankBottom1PressureData : Boardcore::PressureData
{
    explicit OxTankBottom1PressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    OxTankBottom1PressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(OxTankBottom1PressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct N2TankPressureData : Boardcore::PressureData
{
    explicit N2TankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    N2TankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(N2TankPressureData,
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

}  // namespace Motor
