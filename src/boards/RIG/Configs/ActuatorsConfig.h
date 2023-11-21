/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

namespace RIG
{
namespace Config
{
namespace Servos
{
static TIM_TypeDef* const SERVO1_TIMER = TIM4;
static TIM_TypeDef* const SERVO2_TIMER = TIM11;
static TIM_TypeDef* const SERVO3_TIMER = TIM3;
static TIM_TypeDef* const SERVO4_TIMER = TIM10;
static TIM_TypeDef* const SERVO5_TIMER = TIM8;

constexpr Boardcore::TimerUtils::Channel SERVO1_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_2;
constexpr Boardcore::TimerUtils::Channel SERVO2_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr Boardcore::TimerUtils::Channel SERVO3_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr Boardcore::TimerUtils::Channel SERVO4_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;
constexpr Boardcore::TimerUtils::Channel SERVO5_PWM_CH =
    Boardcore::TimerUtils::Channel::CHANNEL_1;

constexpr uint16_t MIN_PULSE = 900;
constexpr uint16_t MAX_PULSE = 2000;

constexpr uint16_t SERVO_TIMINGS_CHECK_PERIOD = 100;
constexpr uint16_t SERVO_CONFIDENCE_TIME      = 500;       // 0.5s
constexpr float SERVO_CONFIDENCE              = 1 / 50.0;  // 2%

constexpr uint32_t DEFAULT_FILLING_OPENING_TIME    = 15000;  // 15s
constexpr uint32_t DEFAULT_VENTING_OPENING_TIME    = 15000;  // 15s
constexpr uint32_t DEFAULT_MAIN_OPENING_TIME       = 6000;   // 6s
constexpr uint32_t DEFAULT_RELEASE_OPENING_TIME    = 10000;  // 10s
constexpr uint32_t DEFAULT_DISCONNECT_OPENING_TIME = 10000;  // 10s

constexpr float DEFAULT_FILLING_MAXIMUM_APERTURE    = 0.97f;
constexpr float DEFAULT_VENTING_MAXIMUM_APERTURE    = 0.80f;
constexpr float DEFAULT_MAIN_MAXIMUM_APERTURE       = 0.87f;
constexpr float DEFAULT_RELEASE_MAXIMUM_APERTURE    = 0.80f;
constexpr float DEFAULT_DISCONNECT_MAXIMUM_APERTURE = 0.110f;
}  // namespace Servos
}  // namespace Config
}  // namespace RIG