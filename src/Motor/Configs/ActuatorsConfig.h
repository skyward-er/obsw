/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <chrono>

namespace Motor
{
namespace Config
{
namespace Servos
{
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

// Pulse width for normal (74 kg) servos
constexpr unsigned int MIN_PULSE = 500;
constexpr unsigned int MAX_PULSE = 2440;

// Pulse width for small (16 kg) servos
constexpr unsigned int SMALL_MIN_PULSE = 900;
constexpr unsigned int SMALL_MAX_PULSE = 2100;

constexpr unsigned int FREQUENCY = 333;

constexpr auto SERVO_BACKSTEP_DELAY   = 500ms;
constexpr float SERVO_BACKSTEP_AMOUNT = 0.02;  // 2%

constexpr auto SAFETY_VENTING_TIMEOUT  = 45min;
constexpr auto SAFETY_VENTING_DURATION = 10min;  // How long to vent

constexpr float OX_VENTING_LIMIT   = 0.9f;
constexpr float MAIN_LIMIT         = 0.9f;
constexpr float NITROGEN_LIMIT     = 0.9f;
constexpr float N2_QUENCHING_LIMIT = 1.0f;

constexpr bool OX_VENTING_FLIPPED   = true;
constexpr bool MAIN_FLIPPED         = true;
constexpr bool NITROGEN_FLIPPED     = true;
constexpr bool N2_QUENCHING_FLIPPED = false;

}  // namespace Servos
}  // namespace Config
}  // namespace Motor
