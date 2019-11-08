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

#include <drivers/HardwareTimer.h>
#include <drivers/pwm/pwm.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include "DeathStack/DeploymentController/Motor/MotorData.h"
#include "config.h"

namespace DeathStackBoard
{

namespace DeploymentConfigs
{

static constexpr uint8_t MAX_EJECTION_ATTEMPTS = 1;

static constexpr int NC_OPEN_TIMEOUT     = 5000;

static const PWM::Timer SERVO_TIMER{
    TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

static constexpr PWMChannel SERVO_CHANNEL = PWMChannel::CH1;

// Servo rest position
static constexpr float SERVO_RESET_POS = 0.77f;
static constexpr float SERVO_WIGGLE_AMPLITUDE = 0.02f;

// Servo position when ejecting the nosecone
static constexpr float SERVO_EJECT_POS = 0.35f;

}  // namespace DeploymentConfigs

}  // namespace DeathStackBoard
