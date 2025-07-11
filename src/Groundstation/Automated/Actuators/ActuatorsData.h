/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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
#include <actuators/stepper/StepperData.h>

namespace Antennas
{

struct StepperConfig
{
    int MICROSTEPPING;
    float STEP_ANGLE;
    float MIN_ANGLE;
    float MAX_ANGLE;
    float MAX_SPEED;
};

struct StepperXData : Boardcore::StepperData
{
    explicit StepperXData() : Boardcore::StepperData() {}

    explicit StepperXData(const Boardcore::StepperData& data)
        : Boardcore::StepperData(data)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StepperXData, EXTEND_DEF(StepperData));
    }
};

struct StepperYData : Boardcore::StepperData
{
    explicit StepperYData() : Boardcore::StepperData() {}

    explicit StepperYData(const Boardcore::StepperData& data)
        : Boardcore::StepperData(data)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StepperYData, EXTEND_DEF(StepperData));
    }
};

}  // namespace Antennas
