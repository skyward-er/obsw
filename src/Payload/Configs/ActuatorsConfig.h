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

#include <units/Frequency.h>

#include <chrono>

namespace Payload
{
namespace Config
{
namespace Actuators
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;

// On Lyra 6-gear planetary multiplier: 100us ~= 1cm

namespace LeftServo
{
constexpr auto ROTATION  = 180.f;  // [deg]
constexpr auto MIN_PULSE = 500us;
constexpr auto MAX_PULSE = 2200us;
}  // namespace LeftServo

namespace RightServo
{
constexpr auto ROTATION  = 180.f;  // [deg]
constexpr auto MIN_PULSE = 2200us;
constexpr auto MAX_PULSE = 500us;
}  // namespace RightServo

namespace StatusLed
{
constexpr auto UPDATE_RATE  = 10_hz;
constexpr auto OK_PERIOD    = 1000ms;
constexpr auto ERROR_PERIOD = 100ms;
}  // namespace StatusLed

namespace Buzzer
{
constexpr auto UPDATE_RATE   = 10_hz;
constexpr auto ARMED_PERIOD  = 500ms;
constexpr auto LANDED_PERIOD = 1000ms;
// PWM parameters
constexpr auto FREQUENCY  = 500_hz;
constexpr auto DUTY_CYCLE = 0.5f;
}  // namespace Buzzer

// Status configs

}  // namespace Actuators
}  // namespace Config
}  // namespace Payload
