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

#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace Motor
{

struct TopTankPressureData : Boardcore::PressureData
{
    explicit TopTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    TopTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TopTankPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct BottomTankPressureData : Boardcore::PressureData
{
    explicit BottomTankPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData{data}
    {
    }

    BottomTankPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(BottomTankPressureData,
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
