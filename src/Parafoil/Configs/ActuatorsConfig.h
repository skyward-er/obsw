/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <drivers/timer/TimerUtils.h>
#include <units/Angle.h>
#include <units/Time.h>

namespace Parafoil
{
namespace Config
{
namespace Actuators
{

/* linter off */ using namespace Boardcore::Units::Time;
/* linter off */ using namespace Boardcore::Units::Angle;

namespace LeftServo
{
static TIM_TypeDef* const TIMER = TIM4;
constexpr Boardcore::TimerUtils::Channel PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

constexpr auto ROTATION  = 180_deg;
constexpr auto MIN_PULSE = 500_us;
constexpr auto MAX_PULSE = 2460_us;
}  // namespace LeftServo

namespace RightServo
{
static TIM_TypeDef* const TIMER = TIM8;
constexpr Boardcore::TimerUtils::Channel PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

constexpr auto ROTATION  = 180_deg;
constexpr auto MIN_PULSE = 2460_us;
constexpr auto MAX_PULSE = 500_us;
}  // namespace RightServo

constexpr auto SERVO_TWIRL_RADIUS = 0.5f;  // [%]

// Status configs

}  // namespace Actuators
}  // namespace Config
}  // namespace Parafoil
