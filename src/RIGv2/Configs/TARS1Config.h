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

#include <units/Frequency.h>

#include <cstddef>
#include <cstdint>

namespace RIGv2
{
namespace Config
{
namespace TARS1
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz SAMPLE_PERIOD         = 100_hz;
constexpr size_t MEDIAN_SAMPLE_NUMBER = 10;

constexpr uint32_t WASHING_OPENING_TIME         = 5000;
constexpr uint32_t WASHING_TIME_DELAY           = 1000;
constexpr uint32_t FILLING_OPENING_TIME         = 900000;
constexpr uint32_t PRESSURE_STABILIZE_WAIT_TIME = 1000;

constexpr int NUM_MASS_STABLE_ITERATIONS = 2;

constexpr float MASS_TOLERANCE     = 0.2;    // [kg]
constexpr float PRESSURE_TOLERANCE = 0.035;  // [bar]

constexpr bool STOP_ON_MASS_STABILIZATION = false;
}  // namespace TARS1
}  // namespace Config
}  // namespace RIGv2
