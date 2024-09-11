
/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

namespace Payload
{

// Wrappers to differentiate static and dynamic pressure for logging

struct StaticPressureData : public Boardcore::PressureData
{
    StaticPressureData() = default;

    explicit StaticPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }
};

struct DynamicPressureData : public Boardcore::PressureData
{
    DynamicPressureData() = default;

    explicit DynamicPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }
};

struct SensorsCalibrationParameter
{
    uint64_t timestamp      = 0;
    float referencePressure = 0;
    float offsetStatic      = 0;
    float offsetDynamic     = 0;

    static std::string header()
    {
        return "timestamp,referencePressure,offsetStatic,offsetDynamic\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << referencePressure << "," << offsetStatic
           << "," << offsetDynamic << "\n";
    }
};

}  // namespace Payload
