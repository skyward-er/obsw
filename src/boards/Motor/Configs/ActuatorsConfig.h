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

#include <interfaces-impl/hwmapping.h>
#include <units/Frequency.h>

namespace Motor
{

namespace Config
{

namespace Servos
{

using namespace Boardcore::Units::Frequency;

// Generic pulse width for all servos
constexpr unsigned int MIN_PULSE = 500;
constexpr unsigned int MAX_PULSE = 2460;

constexpr unsigned int FREQUENCY = 333;

constexpr Hertz SERVO_TIMINGS_CHECK_PERIOD = 10_hz;
constexpr long long SERVO_CONFIDENCE_TIME  = 500;   // 0.5s
constexpr float SERVO_CONFIDENCE           = 0.02;  // 2%

constexpr float VENTING_LIMIT = 0.90f;
constexpr float MAIN_LIMIT    = 0.90f;

constexpr bool VENTING_FLIPPED = true;
constexpr bool MAIN_FLIPPED    = true;

}  // namespace Servos

}  // namespace Config
}  // namespace Motor