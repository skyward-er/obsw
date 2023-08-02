/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <drivers/timer/PWM.h>
#include <drivers/timer/TimerUtils.h>

namespace Main
{
namespace ActuatorsConfig
{
static TIM_TypeDef* const SERVO_ABK_TIMER = TIM3;
static TIM_TypeDef* const SERVO_EXP_TIMER = TIM1;
static TIM_TypeDef* const BUZZER_TIMER    = TIM1;

// TODO change correspondent naming in hwmapping (channel different)
constexpr Boardcore::TimerUtils::Channel SERVO_ABK_CHANNEL =
    Boardcore::TimerUtils::Channel::CHANNEL_2;
constexpr Boardcore::TimerUtils::Channel SERVO_EXP_CHANNEL =
    Boardcore::TimerUtils::Channel::CHANNEL_3;
constexpr Boardcore::TimerUtils::Channel BUZZER_CHANNEL =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

constexpr uint16_t ABK_MIN_PULSE = 900;
constexpr uint16_t ABK_MAX_PULSE = 2000;

// Inverted to invert the servo logic
constexpr uint16_t EXP_MIN_PULSE = 2000;
constexpr uint16_t EXP_MAX_PULSE = 900;

// Buzzer configs
constexpr uint32_t BUZZER_FREQUENCY = 1000;
constexpr float BUZZER_DUTY_CYCLE   = 0.5;

constexpr uint32_t BUZZER_UPDATE_PERIOD = 100;  // [ms]

constexpr uint32_t BUZZER_ARM_PERIOD  = 500;   // [ms]
constexpr uint32_t BUZZER_LAND_PERIOD = 1000;  // [ms]

}  // namespace ActuatorsConfig
}  // namespace Main