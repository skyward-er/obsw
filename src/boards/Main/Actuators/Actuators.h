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

#include <Singleton.h>
#include <actuators/Servo/Servo.h>
#include <common/Mavlink.h>
#include <drivers/timer/PWM.h>
#include <interfaces/gpio.h>
#ifdef HILSimulation
#include "HIL_actuators/HILServo.h"
#endif  // HILSimulation

namespace Main
{

struct Actuators : public Boardcore::Singleton<Actuators>
{
    friend class Boardcore::Singleton<Actuators>;

    miosix::GpioPin cutter1;
    miosix::GpioPin cutter1Backup;

    /**
     * @brief Moves the specified servo to the given position.
     *
     * @param servoId Servo to move.
     * @param percentage Angle to set [0-1].
     * @return True if the the angle was set.
     */
    bool setServo(ServosList servoId, float percentage);

    /**
     * @brief Moves the specified servo to the given position.
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

    float getServoPosition(ServosList servoId);

    float getServoAngle(ServosList servoId);

    void buzzerError();
    void buzzerDisarmed();
    void buzzerArmed();
    void buzzerLanded();
    void buzzerOff();

#ifdef HILSimulation
    void sendToSimulator();
#endif  // HILSimulation

private:
    Actuators();

    void toggleBuzzer();

#ifndef HILSimulation
    Boardcore::Servo servoAirbrakes;
#else   // HILSimulation
    HILServo servoAirbrakes;
#endif  // HILSimulation
    Boardcore::Servo servoExpulsion;

    Boardcore::PWM buzzer;
    int buzzerCounter         = -1;
    int buzzerCurrentOverflow = -1;
    uint8_t buzzerTaskId;
};

}  // namespace Main
