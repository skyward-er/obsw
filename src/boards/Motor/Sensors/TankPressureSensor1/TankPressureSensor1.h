/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <sensors/analog/pressure/AnalogPressureSensor.h>

#include "TankPressureSensor1Data.h"

namespace Boardcore
{

class TankPressureSensor1 final
    : public AnalogPressureSensor<TankPressureSensor1Data>
{
public:
    TankPressureSensor1(std::function<ADCData()> getVoltage,
                        const float maxPressure = 0,
                        const float minPressure = 0, const float coeff = 0)
        : AnalogPressureSensor(getVoltage, (maxPressure - minPressure) * coeff,
                               maxPressure, minPressure),
          coeff(coeff)
    {
    }

private:
    float voltageToPressure(float voltage) override { return voltage * coeff; }

    const float coeff;
};

}  // namespace Boardcore
