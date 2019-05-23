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

// TODO: Update with correct values
static constexpr int MAXIMUM_CUTTING_DURATION = 15 * 1000;

static constexpr int NC_MINIMUM_OPENING_TIME = 4000;
static constexpr int NC_OPEN_TIMEOUT         = 15000;
static constexpr int NC_CLOSE_TIMEOUT        = 15000;

static const MotorDirection MOTOR_OPEN_DIR  = MotorDirection::NORMAL;
static const MotorDirection MOTOR_CLOSE_DIR = MotorDirection::REVERSE;

static const PWM::Timer TIM4_DATA{
    TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

static const PWM::Timer TIM8_DATA{
    TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

static constexpr PWMChannel SERVO_LEFT_CH  = PWMChannel::CH2;
static constexpr PWMChannel SERVO_RIGHT_CH = PWMChannel::CH2;
static constexpr PWMChannel SERVO_KEEL_CH  = PWMChannel::CH1;

// Right servo reset position
static constexpr float SERVO_R_RESET_POS = 0.875f;
// Right servo control position
static constexpr float SERVO_R_CONTROL_POS = 0.80f;

// Keel servo reset position
static constexpr float SERVO_K_RESET_POS = 0.875f;
// Keel servo control position
static constexpr float SERVO_K_CONTROL_POS = 0.80f;

// Left servo reset position
static constexpr float SERVO_L_RESET_POS = 0.125f;
// Left servo control position
static constexpr float SERVO_L_CONTROL_POS = 0.20f;

}  // namespace DeathStackBoard
