/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"
#include "interfaces-impl/hwmapping.h"


namespace DeathStackBoard
{
// clang-format off

// Struct required by the PWM driver to know the specifics of the timer to use
static const PWM::Timer CUTTER_TIM{
    TIM9, 
    &(RCC->APB2ENR), 
    RCC_APB2ENR_TIM9EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)
    };

// clang-format on

// DROGUE --> Right H-Bridge, THCUT1 on theboard
static const PWMChannel CUTTER_CHANNEL_DROGUE = PWMChannel::CH2; // PD12
typedef miosix::actuators::thCut1::ena DrogueCutterEna; // PG2

// MAIN CHUTE --> Left H-Bridge, THCUT2 on theboard
static const PWMChannel CUTTER_CHANNEL_MAIN_CHUTE = PWMChannel::CH2; // PD13
typedef miosix::actuators::thCut2::ena MainChuteCutterEna; //PD11

// PWM Frequency & duty-cycle
static const unsigned int CUTTER_PWM_FREQUENCY = 15000;
// FREQ: 15k
// 0.3: 2.2 s cut -> ni-cr wire broken
// 0.15: 8.5 s cut, too slow
static const float CUTTER_PWM_DUTY_CYCLE       = 0.30;

// Period of time where the IN must be kept low before bringing ENA/INH low
static const int CUTTER_DISABLE_DELAY_MS = 50;
 
} // DeathStackBoard


