/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Alberto Nidasio
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

namespace Motor
{

namespace ActuatorsConfig
{

static TIM_TypeDef* const SERVO_MAIN_TIMER    = TIM8;
static TIM_TypeDef* const SERVO_VENTING_TIMER = TIM8;

constexpr Boardcore::TimerUtils::Channel SERVO_MAIN_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr Boardcore::TimerUtils::Channel SERVO_VENTING_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

constexpr uint16_t MIN_PULSE = 900;
constexpr uint16_t MAX_PULSE = 2000;

constexpr uint16_t SERVO_TIMINGS_CHECK_PERIOD = 100;
constexpr uint16_t SERVO_CONFIDENCE_TIME      = 500;       // 0.5s
constexpr float SERVO_CONFIDENCE              = 1 / 50.0;  // 2%

constexpr uint32_t DEFAULT_MAIN_OPENING_TIME    = 6000;   // 6s
constexpr uint32_t DEFAULT_VENTING_OPENING_TIME = 15000;  // 15s

constexpr float DEFAULT_MAIN_MAXIMUM_APERTURE    = 0.97f;
constexpr float DEFAULT_VENTING_MAXIMUM_APERTURE = 0.35f;

}  // namespace ActuatorsConfig

}  // namespace Motor