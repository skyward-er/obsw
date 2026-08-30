/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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

#include <ActiveObject.h>
#include <common/Events.h>
#include <common/canbus/MotorStatus.h>
#include <events/EventBroker.h>
#include <events/EventHandler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv3
{
class Actuators;
class CanHandler;
class Radio;
class Sensors;

class ValveSequenceController
    : public Boardcore::EventHandler,
      public Boardcore::InjectableWithDeps<Actuators, Sensors, Radio,
                                           CanHandler, Common::MotorStatus>
{
public:
    ValveSequenceController(
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY);

    /**
     * @brief Performs a wiggle on all valves.
     *
     * Opens all valves, checks if their positions exceed the open thresholds,
     * closes them, and verifies they fall below the closed thresholds.
     *
     * @param requestId The request ID to use for the telemetry message
     * response.
     *
     * @return A bitmask mapping the success of each valve (RIG on low byte,
     * Motor on high byte).
     */
    uint16_t wiggleValves(
        uint8_t requestId = Config::Radio::MAV_DEFAULT_REQUEST_ID);

    /**
     * @brief Closes all RIGv3 and Motor valves in a safe, timed sequence.
     *
     * Sequentially closes valves with safety delays, in order to avoid power
     * surges.
     */
    void closeValves();

    void requestAsyncAutomaticWiggle(uint8_t requestId)
    {
        lastRequestId = requestId;
        EventBroker::getInstance().post(WIGGLE_ALL_VALVES,
                                        TOPIC_VALVE_SEQUENCE);
    }

protected:
    void handleEvent(const Boardcore::Event& ev) override;

    uint8_t lastRequestId = Config::Radio::MAV_DEFAULT_REQUEST_ID;
};
}  // namespace RIGv3
