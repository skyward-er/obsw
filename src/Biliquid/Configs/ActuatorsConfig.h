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

#include <chrono>

namespace Biliquid
{
namespace Config
{
namespace Servos
{
/* linter off */ using namespace std::chrono;

// Pulse width for all servos
constexpr unsigned int MIN_PULSE = 500;
constexpr unsigned int MAX_PULSE = 2440;

constexpr unsigned int FREQUENCY = 333;

// Time for the servo to move across its full range
constexpr auto SERVO_FULL_RANGE_TIME  = 500ms;
constexpr float SERVO_BACKSTEP_AMOUNT = 0.02;  // 2%

constexpr float MAIN_OX_MAX_APERTURE   = 1.0;
constexpr float MAIN_FUEL_MAX_APERTURE = 1.0;

constexpr float MAIN_OX_LIMIT   = 0.9;
constexpr float MAIN_FUEL_LIMIT = 0.9;

constexpr bool MAIN_OX_FLIPPED   = true;
constexpr bool MAIN_FUEL_FLIPPED = true;

}  // namespace Servos
}  // namespace Config
}  // namespace Biliquid
