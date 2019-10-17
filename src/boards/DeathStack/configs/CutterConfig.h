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

static constexpr int CUT_DURATION = 5 * 1000;

// PRIMARY --> THCUT1 on theboard
static const PWMChannel CUTTER_CHANNEL_PRIMARY = PWMChannel::CH2; 
typedef miosix::actuators::thCut1::ena PrimaryCutterEna;

// BACKUP --> THCUT2 on theboard
static const PWMChannel CUTTER_CHANNEL_BACKUP = PWMChannel::CH2;
typedef miosix::actuators::thCut2::ena BackupCutterEna;

// PWM Frequency & duty-cycle
static const unsigned int CUTTER_PWM_FREQUENCY = 15000;  // Hz
// Duty cycle to be used during flight to cut the chord
static constexpr float CUTTER_PWM_DUTY_CYCLE = 0.30f;

// Duty cycle to be used during integration, to perform a a non-destructive
// continuity check
static constexpr float CUTTER_TEST_PWM_DUTY_CYCLE = 0.05f;

// Period of time where the IN must be kept low before bringing ENA/INH low
static const int CUTTER_DISABLE_DELAY_MS = 50;

}  // namespace DeathStackBoard
