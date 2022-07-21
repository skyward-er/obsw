/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

namespace Parafoil
{

namespace ActuatorsConfigs
{

// Servo 1

static TIM_TypeDef* const SERVO_1_TIMER = TIM4;
static constexpr Boardcore::TimerUtils::Channel SERVO_1_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

static constexpr float SERVO_1_ROTATION  = 120;
static constexpr float SERVO_1_MIN_PULSE = 900;   // [deg]
static constexpr float SERVO_1_MAX_PULSE = 2100;  // [deg]

// Servo 2

static TIM_TypeDef* const SERVO_2_TIMER = TIM10;
static constexpr Boardcore::TimerUtils::Channel SERVO_2_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

static constexpr float SERVO_2_ROTATION  = 120;
static constexpr float SERVO_2_MIN_PULSE = 2100;  // [deg]
static constexpr float SERVO_2_MAX_PULSE = 900;   // [deg]

}  // namespace ActuatorsConfigs

}  // namespace Parafoil
