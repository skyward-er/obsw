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

#include <chrono>

namespace Payload
{
namespace Config
{
namespace Actuators
{

// clang-format off
// Indent to avoid the linter complaining about using namespace
  using namespace std::chrono;
// clang-format on

namespace LeftServo
{
constexpr auto ROTATION  = 120.f;  // [deg]
constexpr auto MIN_PULSE = 900us;
constexpr auto MAX_PULSE = MIN_PULSE + 10us * static_cast<int>(ROTATION);
}  // namespace LeftServo

namespace RightServo
{
constexpr auto ROTATION  = 120.f;  // [deg]
constexpr auto MIN_PULSE = 2100us;
constexpr auto MAX_PULSE = MIN_PULSE - 10us * static_cast<int>(ROTATION);
}  // namespace RightServo

namespace StatusLed
{
constexpr auto UPDATE_PERIOD = 50ms;
constexpr auto OK_PERIOD     = 1000ms;
constexpr auto ERROR_PERIOD  = 500ms;
}  // namespace StatusLed

namespace Buzzer
{
constexpr auto UPDATE_PERIOD  = 50ms;
constexpr auto ON_LAND_PERIOD = 1000ms;
constexpr auto ARMED_PERIOD   = 500ms;
// PWM parameters
constexpr auto FREQUENCY  = 1000;  // [Hz]
constexpr auto DUTY_CYCLE = 0.5f;
}  // namespace Buzzer

// Status configs

}  // namespace Actuators
}  // namespace Config
}  // namespace Payload
