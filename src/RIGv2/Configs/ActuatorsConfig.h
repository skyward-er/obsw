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

namespace RIGv2
{
namespace Config
{

namespace Servos
{

/* linter off */ using namespace Boardcore::Units::Frequency;

// Generic pulse width for all servos
constexpr unsigned int MIN_PULSE = 500;
constexpr unsigned int MAX_PULSE = 2440;

// Pulse width specific to SERVO 2 (disconnect servo)
// TODO(davide.mor): This actually needs tweaking
constexpr unsigned int SERVO2_MIN_PULSE = 900;
constexpr unsigned int SERVO2_MAX_PULSE = 2100;

constexpr unsigned int FREQUENCY = 333;

constexpr Hertz SERVO_TIMINGS_CHECK_PERIOD = 10_hz;
constexpr long long SERVO_CONFIDENCE_TIME  = 500;   // 0.5s
constexpr float SERVO_CONFIDENCE           = 0.02;  // 2%

constexpr uint32_t DEFAULT_FILLING_OPENING_TIME    = 15000;  // 15s
constexpr uint32_t DEFAULT_VENTING_OPENING_TIME    = 15000;  // 15s
constexpr uint32_t DEFAULT_MAIN_OPENING_TIME       = 6000;   // 6s
constexpr uint32_t DEFAULT_RELEASE_OPENING_TIME    = 10000;  // 10s
constexpr uint32_t DEFAULT_DISCONNECT_OPENING_TIME = 2000;   // 2s

constexpr float DEFAULT_FILLING_MAX_APERTURE    = 1.00f;
constexpr float DEFAULT_VENTING_MAX_APERTURE    = 1.00f;
constexpr float DEFAULT_MAIN_MAX_APERTURE       = 1.00f;
constexpr float DEFAULT_RELEASE_MAX_APERTURE    = 1.00f;
constexpr float DEFAULT_DISCONNECT_MAX_APERTURE = 1.00f;

constexpr float FILLING_LIMIT    = 0.90f;
constexpr float VENTING_LIMIT    = 0.90f;
constexpr float MAIN_LIMIT       = 0.90f;
constexpr float RELEASE_LIMIT    = 0.50f;
constexpr float DISCONNECT_LIMIT = 1.00f;

constexpr bool FILLING_FLIPPED    = true;
constexpr bool VENTING_FLIPPED    = true;
constexpr bool MAIN_FLIPPED       = true;
constexpr bool RELEASE_FLIPPED    = true;
constexpr bool DISCONNECT_FLIPPED = false;

}  // namespace Servos

namespace Actuators
{
static constexpr uint32_t NITROGEN_OPENING_TIME = 5000;  // 5s
}

}  // namespace Config
}  // namespace RIGv2
