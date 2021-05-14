/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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

#include "Constants.h"
#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"

namespace DeathStackBoard
{
namespace AeroBrakesConfigs
{

static const PWM::Timer AB_SERVO_TIMER{
    TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

static constexpr PWMChannel AB_SERVO_PWM_CH = PWMChannel::CH2;

static constexpr uint32_t LOOKS               = 50;
static constexpr uint32_t START_INDEX_OFFSET  = -1;
static constexpr float M                      = 22;   /**< rocket's mass */
static constexpr float D                      = 0.15; /**< rocket's diameter */
static constexpr float S0                     = (PI * D * D) / 4;
static constexpr float RHO                    = 1.225;
static constexpr float Hn                     = 10400;
static constexpr float Kp                     = 55;  // 77;
static constexpr float Ki                     = 5;
static constexpr float Co                     = 340.3;
static constexpr float ALPHA                  = -3.871e-3;
static constexpr float A                      = -1.04034;
static constexpr float B                      = 0.30548;
static constexpr float A_DELTAS               = -9.43386 / 1000;
static constexpr float B_DELTAS               = 19.86779 / 1000;
static constexpr float DELTA_S_AVAILABLE_MIN  = 0;
static constexpr float DELTA_S_AVAILABLE_MAX  = 0.01;
static constexpr float DELTA_S_AVAILABLE_STEP = 0.0005;

static constexpr float AB_SERVO_MAX_POS          = 55;         // deg
static constexpr float AB_SERVO_MIN_POS          = 0;          // deg
static constexpr float AB_SERVO_MAX_RATE         = 60 / 0.2;   // deg/s
static constexpr float AB_SERVO_MIN_RATE         = -60 / 0.2;  // deg/s
static constexpr float AB_SERVO_WIGGLE_AMPLITUDE = 10;  // deg, 0.17 in radians

static constexpr float FILTER_COEFF = 0.9;

static constexpr float UPDATE_TIME = 0.05 * 1000;  // ms -> 20 Hz

static constexpr int SHADOW_MODE_DURATION = 1000;

}  // namespace AeroBrakesConfigs

}  // namespace DeathStackBoard