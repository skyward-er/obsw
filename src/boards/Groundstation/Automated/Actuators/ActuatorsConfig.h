/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

namespace Antennas
{
namespace Config
{
constexpr int HORIZONTAL_MICROSTEPPING = 4;
constexpr float HORIZONTAL_STEP_ANGLE  = 1.8;
constexpr float HORIZONTAL_MULTIPLIER  = 2.2;
constexpr float MIN_ANGLE_HORIZONTAL   = -180.0;
constexpr float MAX_ANGLE_HORIZONTAL   = 180.0;
constexpr float MAX_SPEED_HORIZONTAL   = 0.75;

constexpr int VERTICAL_MICROSTEPPING = 4;
constexpr float VERTICAL_STEP_ANGLE  = 1.8;
constexpr float VERTICAL_MULTIPLIER  = 2.2;
constexpr float MIN_ANGLE_VERTICAL   = 0;
constexpr float MAX_ANGLE_VERTICAL   = 90.0;
constexpr float MAX_SPEED_VERTICAL   = 0.75;

namespace StepperConfig
{
// TIM1_CH4 PA11 AF1
//      |
// TIM3_CH2 PC7  AF2

// TIM4_CH1 PD12 AF2
//      |
// TIM8_CH4 PC9  AF3
static TIM_TypeDef* const SERVO1_PULSE_TIM = TIM3;
static TIM_TypeDef* const SERVO1_COUNT_TIM = TIM1;
static TIM_TypeDef* const SERVO2_PULSE_TIM = TIM4;
static TIM_TypeDef* const SERVO2_COUNT_TIM = TIM8;

constexpr Boardcore::TimerUtils::Channel SERVO1_PULSE_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;
constexpr Boardcore::TimerUtils::Channel SERVO1_COUNT_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_4;
constexpr Boardcore::TimerUtils::Channel SERVO2_PULSE_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr Boardcore::TimerUtils::Channel SERVO2_COUNT_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_4;

constexpr Boardcore::TimerUtils::TriggerSource SERVO1_PULSE_ITR =
    Boardcore::TimerUtils::TriggerSource::ITR0;
constexpr Boardcore::TimerUtils::TriggerSource SERVO1_COUNT_ITR =
    Boardcore::TimerUtils::TriggerSource::ITR2;
constexpr Boardcore::TimerUtils::TriggerSource SERVO2_PULSE_ITR =
    Boardcore::TimerUtils::TriggerSource::ITR3;
constexpr Boardcore::TimerUtils::TriggerSource SERVO2_COUNT_ITR =
    Boardcore::TimerUtils::TriggerSource::ITR2;
}  // namespace StepperConfig
}  // namespace Config
}  // namespace Antennas