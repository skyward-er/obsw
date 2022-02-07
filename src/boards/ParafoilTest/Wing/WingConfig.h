/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */ 
#pragma once

#include <drivers/timer/PWM.h>

using namespace Boardcore;

/**
 * @brief This class defines all the wing configs
 * (E.g. Servos PWM channels, Servos max and min positions, etc..)
 */
namespace ParafoilTestDev
{
    /**
     * ALGORITHM CONFIGURATION
     */
    static const uint32_t   WING_UPDATE_PERIOD = 10; //milliseconds
    static const uint8_t    WING_CONTROLLER_ID = 100; //TODO define a correct ID

    /**
     * SERVOS CONFIGURATIONS
     */

    //16 bit, 45MHz, no DMA timer
    static TIM_TypeDef* WING_SERVO1_TIMER = TIM13;
    static TIM_TypeDef* WING_SERVO2_TIMER = TIM14;

    static const TimerUtils::Channel WING_SERVO1_PWM_CHANNEL = TimerUtils::Channel::CHANNEL_1;
    static const TimerUtils::Channel WING_SERVO2_PWM_CHANNEL = TimerUtils::Channel::CHANNEL_1;

    //Servo dipendent variables
    static const unsigned int WING_SERVO_MIN_PULSE = 1000;
    static const unsigned int WING_SERVO_MAX_PULSE = 2500;
    static const unsigned int WING_SERVO_FREQUENCY = 50;

    static const float WING_SERVO1_MAX_POSITION = 180; //degrees
    static const float WING_SERVO2_MAX_POSITION = 180; //degrees

    static const float WING_SERVO1_MIN_POSITION = 0;   //degrees
    static const float WING_SERVO2_MIN_POSITION = 0;   //degrees

    static const float WING_SERVO1_RESET_POSITION = 90; //degrees
    static const float WING_SERVO2_RESET_POSITION = 90; //degrees
}