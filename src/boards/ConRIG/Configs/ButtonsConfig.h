/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

/**
 * This class specifies the sensors constants that the sensor manager
 * needs to know about every device. For example the sample time is
 * essential to understand how much time a sensor should wait before
 * another sample.
 */

#pragma once

#include <units/Frequency.h>

namespace ConRIG
{
namespace Config
{
namespace Buttons
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz BUTTON_SAMPLE_PERIOD = 50_hz;

constexpr uint8_t GUARD_THRESHOLD =
    5;  // 5 samples to trigger the guard and activate a single button

}  // namespace Buttons
}  // namespace Config

}  // namespace ConRIG