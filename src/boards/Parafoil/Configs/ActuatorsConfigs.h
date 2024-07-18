/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Federico Lolli, Angelo Prete
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

#include "WingConfig.h"

namespace Parafoil
{

namespace ActuatorsConfigs
{

// Left servo
static TIM_TypeDef* const SERVO_1_TIMER = TIM4;
constexpr Boardcore::TimerUtils::Channel SERVO_1_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

constexpr float SERVO_ROTATION = 180;

constexpr float LEFT_SERVO_ROTATION  = SERVO_ROTATION;  // [deg]
constexpr float LEFT_SERVO_MIN_PULSE = 500;             // [us]
constexpr float LEFT_SERVO_MAX_PULSE = 2460;            // [us]

// Right servo
static TIM_TypeDef* const SERVO_2_TIMER = TIM8;
constexpr Boardcore::TimerUtils::Channel SERVO_2_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

constexpr float RIGHT_SERVO_ROTATION  = SERVO_ROTATION;  // [deg]
constexpr float RIGHT_SERVO_MIN_PULSE = 2460;            // [us]
constexpr float RIGHT_SERVO_MAX_PULSE = 500;             // [us]

// Parafoil twirl
constexpr float SERVO_TWIRL_RADIUS = 0.5;  // [%]

}  // namespace ActuatorsConfigs

}  // namespace Parafoil
