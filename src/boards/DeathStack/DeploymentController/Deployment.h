/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "DeathStack/LogProxy/LogProxy.h"
#include "DeploymentData.h"
#include "Motor/MotorDriver.h"
#include "ThermalCutter/Cutter.h"
#include "events/HSM.h"
#include "utils/CircularBuffer.h"

class PinObserver;

namespace DeathStackBoard
{

class DeploymentController : public HSM<DeploymentController>
{
public:
    DeploymentController();
    ~DeploymentController();

    State state_initialization(const Event &ev);

    State state_idle(const Event &ev);

    State state_cuttingDrogue(const Event &ev);
    State state_cuttingMain(const Event &ev);

    State state_openingNosecone(const Event &ev);
    State state_spinning(const Event &ev);
    State state_awaitingDetachment(const Event &ev);
    State state_awaitingOpenTime(const Event &ev);

private:
    /**
     * @brief Logs the DeploymentStatus struct updating the timestamp and the
     * current state
     *
     * @param current_state
     */
    void logStatus(DeploymentCTRLState current_state)
    {
        status.state = current_state;
        logStatus();
    }
    /**
     * @brief Logs the DeploymentStatus struct updating the timestamp
     */
    void logStatus()
    {
        status.timestamp     = miosix::getTick();
        status.cutter_status = cutter.getStatus();
        status.motor_status  = motor.getStatus();

        logger.log(status);
    }

    /**
     * Defer an event to be processed when the state machine goes back to
     * state_idle
     *
     * @param ev The event to be defered.
     */
    void deferEvent(const Event &ev);

    Cutter cutter{};
    DeploymentStatus status{};
    MotorDriver motor{};

    LoggerProxy &logger = *(LoggerProxy::getInstance());

    CircularBuffer<Event, DEFERRED_EVENTS_BUFFER_SIZE> deferred_events;

    uint16_t ev_min_open_time_id = 0;
    uint16_t ev_open_timeout_id = 0;
    uint16_t ev_cut_timeout_id = 0;
};

}  // namespace DeathStackBoard