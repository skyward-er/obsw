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

}  // namespace RIGv3
