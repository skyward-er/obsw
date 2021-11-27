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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Constants.h>
#include <drivers/HardwareTimer.h>
#include <drivers/pwm/pwm.h>

using namespace Boardcore;

namespace PayloadBoard
{

namespace WingConfigs
{

static const PWM::Timer WING_SERVO_1_TIMER{
    TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

static constexpr PWMChannel WING_SERVO_1_PWM_CH = PWMChannel::CH2;

static const PWM::Timer WING_SERVO_2_TIMER{
    TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

static constexpr PWMChannel WING_SERVO_2_PWM_CH = PWMChannel::CH2;

// Wing servo configs
static constexpr float WING_SERVO_MAX_POS = 180.0;  // deg
static constexpr float WING_SERVO_MIN_POS = 0.0;    // deg

static constexpr float WING_SERVO_WIGGLE_AMPLITUDE = 20.0;  // deg

}  // namespace WingConfigs

}  // namespace PayloadBoard