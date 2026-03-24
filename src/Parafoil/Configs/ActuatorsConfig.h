/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <units/Angle.h>
#include <units/Frequency.h>
#include <units/Time.h>

#include <chrono>

namespace Parafoil
{
namespace Config
{
namespace Actuators
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Angle;
/* linter off */ using namespace Boardcore::Units::Time;

// On Lyra 6-gear planetary multiplier: 100us ~= 1cm

namespace LeftServo
{
constexpr auto MIN_PULSE               = 500_us;
constexpr auto MAX_PULSE               = 2440_us;
constexpr auto HERTZ                   = 333_hz;
constexpr auto SCHMITT_THRESHOLD_LOW   = 10_deg;
constexpr auto SCHMITT_THRESHOLD_HIGH  = 10_deg;
constexpr auto HIGH_THRESHOLD_VELOCITY = 1.f;
constexpr auto LOW_THRESHOLD_VELOCITY  = 0.f;
constexpr auto STOP_THRESHOLD_VELOCITY = 0.5f;
constexpr auto INITIAL_ANGLE           = 0.0_deg;
constexpr auto WIGGLE_ANGLE            = 720_deg;
}  // namespace LeftServo

namespace RightServo
{
constexpr auto MIN_PULSE               = 500_us;
constexpr auto MAX_PULSE               = 2440_us;
constexpr auto HERTZ                   = 333_hz;
constexpr auto SCHMITT_THRESHOLD_LOW   = 10_deg;
constexpr auto SCHMITT_THRESHOLD_HIGH  = 10_deg;
constexpr auto HIGH_THRESHOLD_VELOCITY = 1.f;
constexpr auto LOW_THRESHOLD_VELOCITY  = 0.f;
constexpr auto STOP_THRESHOLD_VELOCITY = 0.5f;
constexpr auto INITIAL_ANGLE           = 0.0_deg;
constexpr auto WIGGLE_ANGLE            = -720_deg;
}  // namespace RightServo

constexpr auto SERVO_TWIRL_RADIUS = 0.5f;  // [%]

namespace StatusLed
{
constexpr auto UPDATE_RATE  = 10_hz;
constexpr auto OK_PERIOD    = 1000ms;
constexpr auto ERROR_PERIOD = 100ms;
}  // namespace StatusLed

namespace Buzzer
{
constexpr auto UPDATE_RATE   = 10_hz;
constexpr auto ARMED_PERIOD  = 1000ms;
constexpr auto LANDED_PERIOD = 1000ms;
// PWM parameters
constexpr auto FREQUENCY  = 500_hz;
constexpr auto DUTY_CYCLE = 0.5f;
}  // namespace Buzzer

// Status configs

}  // namespace Actuators
}  // namespace Config
}  // namespace Parafoil
