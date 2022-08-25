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

namespace Main
{

namespace ActuatorsConfigs
{

// Deployment servo

static TIM_TypeDef* const DPL_SERVO_TIMER = TIM4;
static constexpr Boardcore::TimerUtils::Channel DPL_SERVO_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

// TODO: Change rotation with min and max
static constexpr float DPL_SERVO_ROTATION  = 23;   // [deg]
static constexpr float DPL_SERVO_MIN_PULSE = 900;  // [us]
static constexpr float DPL_SERVO_MAX_PULSE =
    900 + 10 * DPL_SERVO_ROTATION;                                // [us]
static constexpr float DPL_SERVO_EJECT_POS = 0;                   // [deg]
static constexpr float DPL_SERVO_RESET_POS = DPL_SERVO_ROTATION;  // [deg]

// Airbrakes servo

TIM_TypeDef* const ABK_SERVO_TIMER = TIM10;
constexpr Boardcore::TimerUtils::Channel ABK_SERVO_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

// TODO: Change rotation with min and max
static constexpr float ABK_SERVO_ROTATION  = 50;    // [deg]
static constexpr float ABK_SERVO_MIN_PULSE = 1410;  // [deg]
static constexpr float ABK_SERVO_MAX_PULSE =
    ABK_SERVO_MIN_PULSE - 10 * ABK_SERVO_ROTATION;  // [us]

}  // namespace ActuatorsConfigs

}  // namespace Main
