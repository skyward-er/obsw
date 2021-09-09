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
#ifdef EUROC
static constexpr float M = 27.0; /**< rocket's mass */
#else
static constexpr float M = 19.2; /**< rocket's mass */
#endif

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
static constexpr float ABK_UPDATE_PERIOD  = 0.05 * 1000;  // ms -> 20 Hz
#endif
static constexpr float ABK_UPDATE_PERIOD_SECONDS = ABK_UPDATE_PERIOD / 1000;

#ifdef EUROC
static constexpr int SHADOW_MODE_DURATION = 7.5 * 1000;
#else
static constexpr int SHADOW_MODE_DURATION = 3.0 * 1000;
#endif

#ifdef EUROC
struct Coefficients
{
    float n000 = 0.4912038462193552;
    float n100 = -1.2829337399401595;
    float n200 = 5.4496525366603175;
    float n300 = -14.373489732912505;
    float n400 = 23.041565019164537;
    float n500 = -20.665098183955564;
    float n600 = 7.781756180733372;
    float n010 = 8.578612850167778;
    float n020 = 139.0917211542655;
    float n110 = 0.7488030083983109;
    float n120 = -143.1155855707458;
    float n210 = 85.43971374816863;
    float n220 = 1012.1982268667119;
    float n310 = -318.72827049284604;
    float n320 = -3465.8287321207386;
    float n410 = 465.76527224155507;
    float n420 = 5018.865244618964;
    float n510 = -216.65938340162148;
    float n520 = -2334.271751434503;
    float n001 = 2.7225033263675867e-06;
};
#else
struct Coefficients
{
    float n000 = 0.5047586920710725;
    float n100 = -1.3293513469123344;
    float n200 = 5.601218191131566;
    float n300 = -14.225368661713572;
    float n400 = 21.663349336681787;
    float n500 = -18.579135819086048;
    float n600 = 6.851826924514666;
    float n010 = 8.436044511643843;
    float n020 = 149.2990640054199;
    float n110 = 3.621048095119312;
    float n120 = -260.80740460918076;
    float n210 = 62.991673668723855;
    float n220 = 1503.2169611333475;
    float n310 = -256.30553427247116;
    float n320 = -4432.423406317357;
    float n410 = 393.01912725674475;
    float n420 = 5881.223355649425;
    float n510 = -187.42852955727568;
    float n520 = -2619.370124692982;
    float n001 = 3.3241142518818432e-06;
};
#endif

const Coefficients coeffs;

}  // namespace AirBrakesConfigs

}  // namespace DeathStackBoard