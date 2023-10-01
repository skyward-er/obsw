/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <stdint.h>

namespace RIG
{
namespace Config
{
namespace TARS
{
constexpr uint16_t TARS_UPDATE_PERIOD       = 10;
constexpr uint8_t TARS_FILTER_SAMPLE_NUMBER = 10;
constexpr float TARS_TARGET_TEMPERATURE     = 25;  // Celsius

constexpr uint32_t TARS_WASHING_OPENING_TIME = 5000;  // [ms]
constexpr uint32_t TARS_WASHING_TIME_DELAY =
    1000;  //[ms] Represents a delay of opening two different valves
constexpr uint32_t TARS_FILLING_OPENING_TIME         = 900000;  // [ms] 15min
constexpr uint32_t TARS_PRESSURE_STABILIZE_WAIT_TIME = 1000;    // [ms] 1s

constexpr uint16_t TARS_NUMBER_MASS_STABLE_ITERATIONS = 2;

constexpr float TARS_MASS_TOLERANCE     = 0.2;    //[kg]
constexpr float TARS_PRESSURE_TOLERANCE = 0.035;  //[bar]
}  // namespace TARS
}  // namespace Config
}  // namespace RIG