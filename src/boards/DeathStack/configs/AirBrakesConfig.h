/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

namespace DeathStackBoard
{

namespace AirBrakesConfigs
{

static const PWM::Timer AB_SERVO_TIMER{
    TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

static constexpr PWMChannel AB_SERVO_PWM_CH = PWMChannel::CH2;

static constexpr int LOOKS                    = 100;
static constexpr int START_INDEX_OFFSET       = -1;
static constexpr float M                      = 22.0; /**< rocket's mass */
static constexpr float D                      = 0.15; /**< rocket's diameter */
static constexpr float S0                     = (PI * D * D) / 4.0;
static constexpr float RHO                    = 1.225;
static constexpr float Hn                     = 10400.0;
static constexpr float Kp                     = 55;
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

static constexpr float AB_SERVO_MAX_POS          = 50;         // deg
static constexpr float AB_SERVO_MIN_POS          = 0;          // deg
static constexpr float AB_SERVO_MAX_RATE         = 60 / 0.2;   // deg/s
static constexpr float AB_SERVO_MIN_RATE         = -60 / 0.2;  // deg/s
static constexpr float AB_SERVO_WIGGLE_AMPLITUDE = 10;         // deg

static constexpr float FILTER_COEFF = 0.9;

static constexpr float ABK_UPDATE_PERIOD         = 0.05 * 1000;  // ms -> 20 Hz
static constexpr float ABK_UPDATE_PERIOD_SECONDS = ABK_UPDATE_PERIOD / 1000;

static constexpr int SHADOW_MODE_DURATION = 7.5 * 1000;

typedef struct
{
    float n000 = 0.496878;
    float n100 = -1.405038;
    float n200 = 6.502618;
    float n300 = -18.314261;
    float n400 = 30.152448;
    float n500 = -26.715700;
    float n600 = 9.711730;
    float n010 = 8.486901;
    float n020 = 141.050398;
    float n110 = 1.233043;
    float n120 = -152.610100;
    float n210 = 81.980768;
    float n220 = 1072.170007;
    float n310 = -309.620754;
    float n320 = -3618.989164;
    float n410 = 455.524477;
    float n420 = 5190.202262;
    float n510 = -212.545170;
    float n520 = -2402.939515;
    float n001 = 0.000003;
} coeffs_t;

const coeffs_t coeffs;

}  // namespace AirBrakesConfigs

}  // namespace DeathStackBoard