/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Federico Lolli
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

namespace Payload
{

namespace ActuatorsConfigs
{

// Left servo
static TIM_TypeDef* const SERVO_1_TIMER = TIM3;
constexpr Boardcore::TimerUtils::Channel SERVO_1_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

constexpr float LEFT_SERVO_ROTATION  = 120;  // [deg]
constexpr float LEFT_SERVO_MIN_PULSE = 900;  // [us]
constexpr float LEFT_SERVO_MAX_PULSE =
    LEFT_SERVO_MIN_PULSE + 10 * LEFT_SERVO_ROTATION;  // [us]

// Right servo
static TIM_TypeDef* const SERVO_2_TIMER = TIM3;
constexpr Boardcore::TimerUtils::Channel SERVO_2_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

constexpr float RIGHT_SERVO_ROTATION  = 120;   // [deg]
constexpr float RIGHT_SERVO_MIN_PULSE = 2100;  // [us]
constexpr float RIGHT_SERVO_MAX_PULSE =
    RIGHT_SERVO_MIN_PULSE - 10 * RIGHT_SERVO_ROTATION;  // [us]

// Buzzer configs
static TIM_TypeDef* const BUZZER_TIMER = TIM1;
constexpr Boardcore::TimerUtils::Channel BUZZER_CHANNEL =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr uint32_t BUZZER_FREQUENCY = 1000;
constexpr float BUZZER_DUTY_CYCLE   = 0.5;

constexpr uint32_t BUZZER_UPDATE_PERIOD = 100;  // [ms]

constexpr uint32_t ROCKET_SS_ERROR_PERIOD = 50;
constexpr uint32_t ROCKET_SS_ARMED_PERIOD = 500;
constexpr uint32_t ROCKET_SS_LAND_PERIOD  = 1000;  // [ms]

}  // namespace ActuatorsConfigs

}  // namespace Payload
