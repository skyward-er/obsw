/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Raul Radu
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

#include <Payload/BoardScheduler.h>

#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Payload
{

/**
 * This class is used by the FailSafe logic to react to
 * the premature opening of the parafoil by the payload
 * Required modules:
 * - Payload::BoardScheduler
 * - Payload::NASController
 */
class VerticalVelocityTrigger : public Boardcore::Module
{
public:
    /**
     * Default constructor for VerticalVelocityTrigger
     * Sets the trigger to disabled by default
     */
    VerticalVelocityTrigger(Boardcore::TaskScheduler* sched);

    /**
     * Starts the module by inserting it in the BoardScheduler
     */
    bool start();

    /**
     * Enables the Vertical Velocity Trigger
     */
    void enable();

    /**
     * Disables the Vertical Velocity Trigger
     */
    void disable();

    /**
     * Checks if the trigger is enabled
     * @return true if the trigger is enabled
     * @return false if the trigger is not enabled
     */
    bool isActive();

private:
    /**
     * This method is called every
     * FailSafe::FAILSAFE_VERTICAL_VELOCITY_TRIGGER_PERIOD
     * and reads the state from the NAS and when the velocity is below
     * the threshold for some confidence level it posts the event
     * FLIGHT_FAILSAFE_TRIGGERED in topic TOPIC_FLIGHT
     */
    void update();

    std::atomic<bool> running;

    // Number of times that the algorithm detects to be slower than the velocity
    // threshold
    std::atomic<int> confidence;

    Boardcore::TaskScheduler* scheduler;
};

}  // namespace Payload
