/* Copyright (c) 2025 Skyward Experimental Rocketry
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

#include <units/Frequency.h>

#include <chrono>

namespace RIGv2
{
namespace Config
{
namespace Servos
{
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace std::chrono;

// Pulse width for normal (74 kg) servos
constexpr unsigned int MIN_PULSE = 500;
constexpr unsigned int MAX_PULSE = 2440;

// Pulse width for small (16 kg) servos
constexpr unsigned int SMALL_MIN_PULSE = 900;
constexpr unsigned int SMALL_MAX_PULSE = 2100;

constexpr unsigned int FREQUENCY = 333;

constexpr auto SERVO_BACKSTEP_DELAY   = 500ms;
constexpr float SERVO_BACKSTEP_AMOUNT = 0.02;  // 2%

constexpr uint32_t DEFAULT_OX_FIL_OPENING_TIME = 15000;
constexpr uint32_t DEFAULT_OX_REL_OPENING_TIME = 10000;
constexpr uint32_t DEFAULT_OX_DET_OPENING_TIME = 2000;
constexpr uint32_t DEFAULT_N2_FIL_OPENING_TIME = 15000;
constexpr uint32_t DEFAULT_N2_REL_OPENING_TIME = 10000;
constexpr uint32_t DEFAULT_N2_DET_OPENING_TIME = 2000;
constexpr uint32_t DEFAULT_NITR_OPENING_TIME   = 600000;
constexpr uint32_t DEFAULT_OX_VEN_OPENING_TIME = 15000;
constexpr uint32_t DEFAULT_N2_QUE_OPENING_TIME = 15000;
constexpr uint32_t DEFAULT_MAIN_OPENING_TIME   = 6000;

constexpr float DEFAULT_OX_FIL_MAX_APERTURE = 1.0;
constexpr float DEFAULT_OX_REL_MAX_APERTURE = 0.55;
constexpr float DEFAULT_OX_DET_MAX_APERTURE = 1.0;
constexpr float DEFAULT_N2_FIL_MAX_APERTURE = 1.0;
constexpr float DEFAULT_N2_REL_MAX_APERTURE = 0.55;
constexpr float DEFAULT_N2_DET_MAX_APERTURE = 1.0;
constexpr float DEFAULT_NITR_MAX_APERTURE   = 1.0;
constexpr float DEFAULT_OX_VEN_MAX_APERTURE = 1.0;
constexpr float DEFAULT_N2_QUE_MAX_APERTURE = 1.0;
constexpr float DEFAULT_MAIN_MAX_APERTURE   = 1.0;

constexpr float OX_FIL_LIMIT = 0.9;
constexpr float OX_REL_LIMIT = 0.9;
constexpr float OX_DET_LIMIT = 1.0;
constexpr float N2_3W_LIMIT  = 1.0;
constexpr float N2_FIL_LIMIT = 1.0;
constexpr float N2_REL_LIMIT = 0.9;
constexpr float N2_DET_LIMIT = 1.0;
constexpr float NITR_LIMIT   = 0.9;
constexpr float OX_VEN_LIMIT = 0.9;
constexpr float N2_QUE_LIMIT = 0.9;
constexpr float MAIN_LIMIT   = 0.9;

constexpr bool OX_FIL_FLIPPED = true;
constexpr bool OX_REL_FLIPPED = true;
constexpr bool OX_DET_FLIPPED = false;
constexpr bool N2_3W_FLIPPED  = true;
constexpr bool N2_FIL_FLIPPED = false;
constexpr bool N2_REL_FLIPPED = false;
constexpr bool N2_DET_FLIPPED = false;
constexpr bool NITR_FLIPPED   = true;
constexpr bool OX_VEN_FLIPPED = true;
constexpr bool N2_QUE_FLIPPED = false;
constexpr bool MAIN_FLIPPED   = true;

}  // namespace Servos
}  // namespace Config
}  // namespace RIGv2
