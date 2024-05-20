/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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
#include <common/Events.h>
#include <common/MavlinkGemini.h>
#include <events/EventBroker.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIG
{
class Actuators : public Boardcore::Module
{
public:
    /**
     * @brief Construct a new Actuators object
     *
     * @param sched The scheduler to respect the atomic timings in case of TARS0
     * engaged
     */
    Actuators(Boardcore::TaskScheduler* sched);

    /**
     * @brief Enables all the servos PWMs and adds to the scheduler a periodic
     * method to check whether the single servos time expired
     */
    [[nodiscard]] bool start();

    /**
     * @brief Wiggles the requested servo for 1 second
     *
     * @param servo The Mavlink requested servo
     * @returns False if the servo does not appear in the list
     */
    bool wiggleServo(ServosList servo);

    /**
     * @brief Get the specified Servo's Position
     *
     * @param servo The Mavlink requested servo
     * @return float The servo position in normalized notation [0-1]
     */
    float getServoPosition(ServosList servo);

    /**
     * @brief Toggles the servo valve passed via parameter. It sets the timings
     * such that the valve opens for a certain amount of time, otherwise, if the
     * valve is already open it closes it.
     *
     * @param servo The servo valve to toggle
     */
    void toggleServo(ServosList servo);

    /**
     * @brief Opens the servo valve passed via parameter for a certain amount of
     * time. If the valve is already open it closes it.
     *
     * @param servo The servo valve to open
     * @param time The time in [ms]
     */
    void openServoAtomic(ServosList servo, uint32_t time);

    /**
     * @brief Closes all the servo valves
     */
    void closeAllServo();

    /**
     * @brief Sets a servo maximum aperture in normalized form. [0,1]
     *
     * @param servo The servo to which set the aperture
     * @param aperture The aperture in [0,1]
     */
    void setMaximumAperture(ServosList servo, float aperture);

    /**
     * @brief Set the timing of a certain valve.
     * @note THE TIMING IS IN [ms]
     *
     * @param servo The servo to which set the timing
     * @param time The timing in [ms]
     */
    void setTiming(ServosList servo, uint32_t time);

    /**
     * @brief Get the Timing of the passed servo
     *
     * @param servo The servo whose timing is needed
     * @return uint32_t The timing in [ms]
     */
    uint32_t getTiming(ServosList servo);

private:
    /**
     * @brief Set the Servo's position to the parameter one
     *
     * @param servo The servo to move
     * @param position A normalized position [0, 1]
     */
    void setServoPosition(ServosList servo, float position);

    Boardcore::Servo* getServo(ServosList servo);

    /**
     * @brief Checks that the atomic timings for the servos didn't expire and
     * sets the positioning according to them.
     * @note Utilizes also the variable setFlag to understand when the position
     * was set the last time. If greater than CONSTANT seconds ago, the function
     * sets a little offset to the servo to avoid power angriness.
     */
    void checkTimings();

    // Create the list of timings for every servo
    uint64_t timings[ServosList::ServosList_ENUM_END] = {0};  // [ms]

    // This set of flags helps the controller to know when the servo have been
    // set, in order to change slightly their angle after CONSTANT time, to
    // avoid over consumption
    uint64_t setFlag[ServosList::ServosList_ENUM_END]      = {0};  // [ms]
    float openings[ServosList::ServosList_ENUM_END]        = {1};
    uint64_t openingTimes[ServosList::ServosList_ENUM_END] = {
        100000};  // Default 100s [ms]

    // This set represents the events to throw at opening/closing of valves
    uint8_t openingEvents[ServosList::ServosList_ENUM_END] = {0};
    uint8_t closingEvents[ServosList::ServosList_ENUM_END] = {0};

    Boardcore::TaskScheduler* scheduler = nullptr;

    Boardcore::Servo* servo1;
    Boardcore::Servo* servo2;
    Boardcore::Servo* servo3;
    Boardcore::Servo* servo4;
    Boardcore::Servo* servo5;
};
}  // namespace RIG