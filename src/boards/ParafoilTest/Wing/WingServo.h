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

#include <common/ServoInterface.h>
#include <drivers/servo/Servo.h>

using namespace Boardcore;

namespace ParafoilTestDev
{
    class WingServo : public ServoInterface
    {
    public:

        /**
         * @brief Construct a new Wing Servo object
         * It will use the min position as reset
         * 
         * @param timer The servo timer that needs to be used
         * @param pwmChannel The servo pwm port used
         * @param minPosition The min allowed servo position(degrees)
         * @param maxPosition The max allowed servo position(degrees)
         */
        WingServo(TIM_TypeDef* timer, TimerUtils::Channel pwmChannel, float minPosition, float maxPosition);

        /**
         * @brief Construct a new Wing Servo object
         * 
         * @param timer The servo timer that needs to be used
         * @param pwmChannel The servo pwm port used
         * @param minPosition The min allowed servo position(degrees)
         * @param maxPosition The max allowed servo position(degrees)
         * @param resetPosition The position where the servo is when it resets(degrees)
         */
        WingServo(TIM_TypeDef* timer, TimerUtils::Channel pwmChannel, float minPosition, float maxPosition, float resetPosition);

        /**
         * @brief Destroy the Wing Servo object
         */
        ~WingServo();

        void enable() override;

        void disable() override;

        /**
         * @brief Execute a self test with the servo
         */
        void selfTest() override;

    private:
        Servo* servo;

    protected:
        /**
         * @brief Set the servo position
         * @param angle sevo position(degrees)
         */
        void setPosition(float angle) override;

        /**
         * @brief Method to convert the angle passed by the user to an
         * useful angle for the servo interface
         * @param angle The angle(degrees)
         * @return float The converted angle
         */
        float preprocessPosition(float angle) override;
    };
}