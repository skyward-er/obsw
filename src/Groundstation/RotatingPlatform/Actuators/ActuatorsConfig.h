/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include "ActuatorsData.h"

namespace RotatingPlatform
{
namespace Config
{

static const RotatingPlatform::StepperConfig stepperXConfig{
    .MICROSTEPPING    = 4,
    .STEP_ANGLE       = 1.8,
    .HAS_ANGLE_LIMITS = false,
    .MIN_ANGLE        = 0,
    .MAX_ANGLE        = 0,
    .MAX_SPEED        = 0.105 * 2.2,  //<[RPS] * multiplier (transmission)
};

namespace StepperSettings
{
// TIM1_CH1 PA8 AF1 Stepper H step
//      |
// TIM3_CH2 PC7  AF2 Stepper H count

static TIM_TypeDef* const SERVO1_PULSE_TIM = TIM1;
static TIM_TypeDef* const SERVO1_COUNT_TIM = TIM3;
static TIM_TypeDef* const SERVO2_PULSE_TIM = TIM4;
static TIM_TypeDef* const SERVO2_COUNT_TIM = TIM8;

constexpr Boardcore::TimerUtils::Channel SERVO1_PULSE_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr Boardcore::TimerUtils::Channel SERVO1_COUNT_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;

constexpr Boardcore::TimerUtils::TriggerSource SERVO1_PULSE_ITR =
    Boardcore::TimerUtils::TriggerSource::ITR2;
constexpr Boardcore::TimerUtils::TriggerSource SERVO1_COUNT_ITR =
    Boardcore::TimerUtils::TriggerSource::ITR0;

}  // namespace StepperSettings
}  // namespace Config
}  // namespace RotatingPlatform
