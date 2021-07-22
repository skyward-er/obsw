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

namespace CutterConfig
{

static const PWM::Timer CUTTER_TIM{
    TIM9, &(RCC->APB2ENR), RCC_APB2ENR_TIM9EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

// PRIMARY --> THCUT1 on theboard
static const PWMChannel CUTTER_CHANNEL_PRIMARY = PWMChannel::CH2;
typedef miosix::actuators::nosecone::thCut1::ena PrimaryCutterEna;

// BACKUP --> THCUT2 on theboard
static const PWMChannel CUTTER_CHANNEL_BACKUP = PWMChannel::CH2;
typedef miosix::actuators::nosecone::thCut2::ena BackupCutterEna;

static constexpr int CUT_DURATION      = 5 * 1000;
static constexpr int CUT_TEST_DURATION = 1 * 1000;

static const unsigned int PRIMARY_CUTTER_PWM_FREQUENCY = 10000;  // Hz
static constexpr float PRIMARY_CUTTER_PWM_DUTY_CYCLE   = 0.45f;
static const unsigned int BACKUP_CUTTER_PWM_FREQUENCY  = 10000;  // Hz
static constexpr float BACKUP_CUTTER_PWM_DUTY_CYCLE    = 0.45f;
static constexpr float CUTTER_TEST_PWM_DUTY_CYCLE      = .1;

}  // namespace CutterConfig

}  // namespace DeathStackBoard