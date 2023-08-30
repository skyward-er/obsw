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
#include <drivers/timer/PWM.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
// TODO verify if a mutex is better (low contention)
class Actuators : public Boardcore::Module
{
public:
    Actuators(Boardcore::TaskScheduler* sched);

    /**
     * @brief Enables all the servos
     */
    bool start();

    /**
     * @brief Sets the servo passed servo position in normalized value [0-1]
     */
    void setServoPosition(ServosList servo, float position);

    /**
     * @brief Sets the servo passed servo position inside the CAN structure
     */
    void setCANServoPosition(ServosList servo, float position);

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

    // Setters for buzzer state
    void setBuzzerArm();
    void setBuzzerLand();
    void setBuzzerOff();

private:
    /**
     * @brief Returns the selected servo pointer
     */
    Boardcore::Servo* getServo(ServosList servo);

    /**
     * @brief Automatic called method to update the buzzer status
     */
    void updateBuzzer();

    // Connected servos
    Boardcore::Servo* servoAbk = nullptr;
    Boardcore::Servo* servoExp = nullptr;

    // Buzzer
    Boardcore::PWM* buzzer;

    // Status LED state
    bool ledState = false;

    // Counter that enables and disables the buzzer
    uint32_t buzzerCounter = 0;

    // Upper limit of the buzzer counter
    uint32_t buzzerCounterOverflow = 0;

    // Can set servo positions
    float CANPositions[ServosList::ServosList_ENUM_END];

    // Scheduler for buzzer
    Boardcore::TaskScheduler* scheduler = nullptr;

    // Access to all the actuators mutex, instead of PauseKernel lock to avoid
    // low priority stuff locking the whole kernel and let the component respect
    // the priority
    miosix::FastMutex mutex;

    // Debug logger
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Actuators");
};
}  // namespace Main