/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <actuators/Servo/Servo.h>
#include <common/Mavlink.h>
#include <diagnostic/PrintLogger.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
// TODO verify if a mutex is better (low contention)
class Actuators : public Boardcore::Module
{
public:
    // TODO manage in case the scheduler for the buzzer and status led
    Actuators();

    /**
     * @brief Enables all the servos
     */
    bool start();

    /**
     * @brief Sets the servo passed servo position in normalized value [0-1]
     */
    void setServoPosition(ServosList servo, float position);

    /**
     * @brief Wiggles the passed servo for 1 second
     * @note The method is blocking during the second of opening
     */
    void wiggleServo(ServosList servo);

    /**
     * @brief Get the passed servo's position [0-1]
     */
    float getServoPosition(ServosList servo);

    /**
     * @brief Turns on the cameras
     */
    void camOn();

    /**
     * @brief Turns off the cameras
     */
    void camOff();

    /**
     * @brief Toggles the LED state
     */
    void toggleLed();

private:
    /**
     * @brief Returns the selected servo pointer
     */
    Boardcore::Servo* getServo(ServosList servo);

    // Connected servos
    Boardcore::Servo* servoAbk = nullptr;
    Boardcore::Servo* servoExp = nullptr;

    // Status LED state
    bool ledState = false;

    // Debug logger
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Actuators");
};
}  // namespace Main