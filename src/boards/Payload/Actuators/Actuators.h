/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Federico Lolli
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

#include <actuators/Servo/Servo.h>
#include <common/Mavlink.h>
#include <interfaces/gpio.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Payload
{

struct Actuators : public Boardcore::Module
{
    explicit Actuators(Boardcore::TaskScheduler* sched);

    [[nodiscard]] bool start();

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

    /**
     * @brief Enables the specified servo.
     *
     * @param servoId Servo to enable.
     * @return True if the servo was enabled.
     * @return False if the servoId is invalid.
     */
    bool enableServo(ServosList servoId);

    /**
     * @brief Disables the specified servo.
     *
     * @param servoId Servo to disable.
     * @return True if the servo was disabled.
     * @return False if the servoId is invalid.
     */
    bool disableServo(ServosList servoId);

    /**
     * @brief Returns the current position of the specified servo.
     *
     * @param servoId Servo to read.
     * @return float current Servo position in range [0-1], 0 if the servoId is
     * invalid.
     */
    float getServoPosition(ServosList servoId);

    /**
     * @brief Returns the current angle of the specified servo.
     *
     * @param servoId Servo to read.
     * @return float current Servo angle in range [0-180], 0 if the servoId is
     * invalid.
     */
    float getServoAngle(ServosList servoId);

    void cuttersOn();
    void cuttersOff();

    void camOn();
    void camOff();

    void ledArmed();
    void ledDisarmed();
    void ledError();
    void ledOff();

private:
    void toggleLed();

    TaskScheduler* scheduler;
    Boardcore::Servo leftServo;
    Boardcore::Servo rightServo;

    enum BlinkState
    {
        LED_OFF,
        LED_ARMED,
        LED_DISARMED,
        LED_ERROR
    };

    // mutexes
    miosix::FastMutex leftServoMutex;
    miosix::FastMutex rightServoMutex;
    miosix::FastMutex ledMutex;

    bool ledState          = false;
    BlinkState blinkState  = LED_OFF;
    uint8_t ledArmedTaskId = 0;
    uint8_t ledErrorTaskId = 0;
};

}  // namespace Payload
