/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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
#include <drivers/timer/TimestampTimer.h>
#include <utils/Constants.h>

namespace PayloadBoard
{

namespace WingConfigs
{

static TIM_TypeDef* const WING_SERVO_1_TIMER = TIM8;
static constexpr Boardcore::TimerUtils::Channel WING_SERVO_1_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

static const TIM_TypeDef* WING_SERVO_2_TIMER = TIM4;
static constexpr Boardcore::TimerUtils::Channel WING_SERVO_2_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

// Wing servo configs
static constexpr float WING_SERVO_MAX_POS = 180.0;  // deg
static constexpr float WING_SERVO_MIN_POS = 0.0;    // deg

static constexpr float WING_SERVO_WIGGLE_AMPLITUDE = 20.0;  // deg

}  // namespace WingConfigs

}  // namespace PayloadBoard
