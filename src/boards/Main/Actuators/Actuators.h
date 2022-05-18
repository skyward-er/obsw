/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/Radio/Mavlink.h>
#include <Singleton.h>
#include <actuators/Servo/Servo.h>
#include <interfaces/gpio.h>

namespace Main
{

struct Actuators : public Boardcore::Singleton<Actuators>
{
    friend class Boardcore::Singleton<Actuators>;

    miosix::GpioPin led1;
    miosix::GpioPin led2;
    miosix::GpioPin led3;

    miosix::GpioPin cutter;

    /**
     * @brief Moves the specified servo to the given position
     *
     * @param servoId Servo to move.
     * @param angle Angle to set [degree].
     * @return True if the the angle was set.
     */
    bool setServoAngle(ServosList servoId, float angle);

    /**
     * @brief Wiggles the servo for few seconds.
     *
     * @param servoId Servo to move.
     * @return true
     * @return false
     */
    bool wiggleServo(ServosList servoId);

    bool enableServo(ServosList servoId);

    bool disableServo(ServosList servoId);

private:
    Actuators();

    Boardcore::Servo servoAirbrakes;
    Boardcore::Servo servoExpulsion;
};

}  // namespace Main
