/* Copyright (c) 2018-2026 Skyward Experimental Rocketry
 * Author: Raul Radu
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
#include <units/Angle.h>

namespace Common
{
class UnlimitedAngle
{
    using Radian = Boardcore::Units::Angle::Radian;
    using Degree = Boardcore::Units::Angle::Degree;

public:
    UnlimitedAngle() : rounds(0), lastReading(0.0) {}

    void setInitialState(Radian reading) { lastReading = reading; }

    Radian getUpdatedAngle(Radian reading)
    {
        auto angleDiff = reading - lastReading;

        if (angleDiff > threshold)
            rounds--;
        else if (angleDiff < -threshold)
            rounds++;

        lastReading = reading;
        return reading + Radian((Degree(360.0 * rounds)));
    }

    Radian getLastReading() { return lastReading; }

private:
    const Radian threshold = Radian(Degree(320.0));
    int rounds;
    Radian lastReading;
};
}  // namespace Common
