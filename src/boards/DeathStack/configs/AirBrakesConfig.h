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
static constexpr float M                 = 18.362;       /**< rocket's mass */
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
// servo angle at which airbrakes exposed area is 50% of the maximum one
static constexpr float AB_SERVO_HALF_AREA_POS = 17.18;  // deg
// airbrakes servo rate : datasheet says 60/0.13, decreased for robustness
static constexpr float AB_SERVO_MAX_RATE         = 15.0 / 0.1;   // deg/s
static constexpr float AB_SERVO_MIN_RATE         = -15.0 / 0.1;  // deg/s
static constexpr float AB_SERVO_WIGGLE_AMPLITUDE = 10.0;         // deg

#ifdef ROCCARASO
// At Roccaraso airbrakes opened at 100%, then 50% and then 0%
// Each opening is kept for 3 seconds (3000 ms)
static constexpr float AB_OPENING_TIMEOUT = 3 * 1000;  // [ms]
#endif

// Control algorithm configs
// static constexpr int LOOKS              = 150;
static constexpr int START_INDEX_OFFSET = -1;
static constexpr float FILTER_COEFF     = 0.9;

#ifdef HARDWARE_IN_THE_LOOP
static constexpr float ABK_UPDATE_PERIOD = 0.1 * 1000;  // ms -> 10 Hz
#else
static constexpr float ABK_UPDATE_PERIOD = 0.05 * 1000;  // ms -> 20 Hz
#endif

static constexpr float ABK_UPDATE_PERIOD_SECONDS = ABK_UPDATE_PERIOD / 1000;

#ifdef EUROC
static constexpr int SHADOW_MODE_DURATION = 6.2 * 1000;  // 0.8 mach
static constexpr int AIRBRAKES_ACTIVATION_AFTER_SHADOW_MODE =
    1.1 * 1000;  // 0.7 mach
#else
static constexpr int SHADOW_MODE_DURATION = 3.5 * 1000;
#endif

#ifdef EUROC
struct Coefficients
{
    float n000 = 0.48691724895766536;
    float n100 = -1.0201967915455747;
    float n200 = 3.1601195121881624;
    float n300 = -5.207372520459633;
    float n400 = 5.348364517420158;
    float n500 = -4.6411809574995235;
    float n600 = 2.410373760485268;
    float n010 = 8.065292810990796;
    float n020 = 143.73493969843133;
    float n110 = 1.231780014534557;
    float n120 = -130.29216244392964;
    float n210 = 68.58740070791052;
    float n220 = 1073.263262818047;
    float n310 = -259.68709984180873;
    float n320 = -3750.7093392408874;
    float n410 = 385.3808010115668;
    float n420 = 5464.924806439974;
    float n510 = -180.76701763197605;
    float n520 = -2554.179469197033;
    float n001 = 2.7555579144973302e-06;
};
#else
struct Coefficients
{
    float n000 = 0.47885644032626357;
    float n100 = -1.3034714816627486;
    float n200 = 5.855250857341603;
    float n300 = -15.804467871608331;
    float n400 = 25.101568926578267;
    float n500 = -21.807834163413037;
    float n600 = 7.9441228817993546;
    float n010 = 8.332725338197339;
    float n020 = 150.92179004169594;
    float n110 = 4.450607930313118;
    float n120 = -261.9317740122675;
    float n210 = 62.374798498326285;
    float n220 = 1402.273850221572;
    float n310 = -261.26595308714985;
    float n320 = -4020.2954140409474;
    float n410 = 402.46390018341435;
    float n420 = 5332.223039294196;
    float n510 = -192.02443481902043;
    float n520 = -2381.855640881901;
    float n001 = 3.0688005499541758e-06;
};
#endif

const Coefficients coeffs;

}  // namespace AirBrakesConfigs

}  // namespace DeathStackBoard