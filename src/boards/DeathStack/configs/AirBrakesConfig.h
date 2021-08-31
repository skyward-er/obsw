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

// Rocket's parameters
static constexpr float M        = 22.0; /**< rocket's mass */
static constexpr float D        = 0.15; /**< rocket's diameter */
static constexpr float S0       = (PI * D * D) / 4.0;
static constexpr float RHO      = 1.225;
static constexpr float Hn       = 10400.0;
static constexpr float Co       = 340.3;
static constexpr float ALPHA    = -3.871e-3;
static constexpr float A        = -1.04034;
static constexpr float B        = 0.30548;
static constexpr float A_DELTAS = -9.43386 / 1000;
static constexpr float B_DELTAS = 19.86779 / 1000;
static constexpr float S_MIN    = 0.0;
static constexpr float S_MAX    = 0.01;
static constexpr float S_STEP   = 0.0005;

// PID configs
static constexpr float Kp = 40;
static constexpr float Ki = 5;

// Airbrakes servo configs
static constexpr float AB_SERVO_MAX_POS = 50.0;  // deg
static constexpr float AB_SERVO_MIN_POS = 0.0;   // deg
// airbrakes servo rate : datasheet says 60/0.13, increased for robustness
static constexpr float AB_SERVO_MAX_RATE         = 15.0 / 0.1;   // deg/s
static constexpr float AB_SERVO_MIN_RATE         = -15.0 / 0.1;  // deg/s
static constexpr float AB_SERVO_WIGGLE_AMPLITUDE = 10.0;         // deg

// Control algorithm configs
// static constexpr int LOOKS              = 150;
static constexpr int START_INDEX_OFFSET = -1;
static constexpr float FILTER_COEFF     = 0.85;
#ifdef HARDWARE_IN_THE_LOOP
static constexpr float ABK_UPDATE_PERIOD = 0.1 * 1000;  // ms -> 10 Hz
#else
static constexpr float ABK_UPDATE_PERIOD = 0.05 * 1000;  // ms -> 20 Hz
#endif
static constexpr float ABK_UPDATE_PERIOD_SECONDS = ABK_UPDATE_PERIOD / 1000;
static constexpr int SHADOW_MODE_DURATION        = 7.5 * 1000;

struct Coefficients
{
    float n000 = 0.4968777393871292;
    float n100 = -1.4050375007975426;
    float n200 = 6.502618436645153;
    float n300 = -18.3142608989424;
    float n400 = 30.152447999970377;
    float n500 = -26.715700256336287;
    float n600 = 9.711730306158518;
    float n010 = 8.48690052994929;
    float n020 = 141.05039771743526;
    float n110 = 1.2330425934312637;
    float n120 = -152.6100996769378;
    float n210 = 81.98076783376676;
    float n220 = 1072.1700065871796;
    float n310 = -309.62075391969813;
    float n320 = -3618.9891638915415;
    float n410 = 455.5244772537147;
    float n420 = 5190.202261565708;
    float n510 = -212.545170192307;
    float n520 = -2402.939514788227;
    float n001 = 2.8334092348814035e-06;
};

const Coefficients coeffs;

}  // namespace AirBrakesConfigs

}  // namespace DeathStackBoard