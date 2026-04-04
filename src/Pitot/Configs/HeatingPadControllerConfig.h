/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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

#include <chrono>
#include <units/Frequency.h>


namespace Pitot
{
    namespace Config
    {
        namespace HeatingPadController
        {
            /* linter-off */ using namespace std::chrono_literals;
            /* linter-off */ using namespace Boardcore::Units::Frequency;

            constexpr auto UPDATE_RATE = 10_hz;

            // Schmitt Trigger parameters
            constexpr auto TARGET_TEMPERATURE = 308.15f; // K, 35°C
            constexpr auto THRESHOLD_LOW = 303.15f; // K, 30°C
            constexpr auto THRESHOLD_HIGH = 313.15f; // K, 40°C
            
        }  // namespace HeatingPadController
    }  // namespace Config
} // namespace Pitot

