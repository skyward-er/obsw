/* Copyright (c) 2025 Skyward Experimental Rocketry
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

#include <units/Frequency.h>

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace RIGv2
{
namespace Config
{
namespace TARS3
{
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz SAMPLE_PERIOD         = 100_hz;
constexpr size_t MEDIAN_SAMPLE_NUMBER = 10;

// Account for ~300ms of servo movement time
constexpr auto WAIT_BETWEEN_CYCLES = 1500ms;

// Cold refueling parameters
constexpr float PRESSURE_TARGET = 35.4f;  // [bar]
constexpr float MASS_TARGET     = 6.5f;   // [kg]
// Threshold to determine if the pressure is near the target
constexpr float NEAR_TARGET_PRESSURE_THRESHOLD = 5.0f;  // [bar]

constexpr auto FILLING_TIME = 2000ms;
constexpr auto VENTING_TIME = 1000ms;

constexpr auto FILLING_TIME_NEAR = 1000ms;
constexpr auto VENTING_TIME_NEAR = 500ms;

}  // namespace TARS3
}  // namespace Config
}  // namespace RIGv2
