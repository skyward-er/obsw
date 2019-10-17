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

#include <drivers/servo/servo.h>
#include "DeathStack/LoggerService/LoggerService.h"
#include "DeathStack/System/StackLogger.h"
#include "DeathStack/configs/DeploymentConfig.h"
#include "DeploymentData.h"
#include "Motor/MotorDriver.h"
#include "ThermalCutter/Cutter.h"
#include "events/HSM.h"
#include "utils/CircularBuffer.h"

class PinObserver;

namespace DeathStackBoard
{

/**
 * @brief Deployment Controller State Machine
 */
class DeploymentController : public HSM<DeploymentController>
{
public:
    DeploymentController(Cutter &cutter, Servo &ejection_servo);
    ~DeploymentController();

    State state_initialization(const Event &ev);

    State state_idle(const Event &ev);

    State state_cuttingPrimary(const Event &ev);
    State state_cuttingBackup(const Event &ev);

    State state_testingPrimary(const Event &ev);
    State state_testingBackup(const Event &ev);

    State state_ejectingNosecone(const Event &ev);
    State state_movingServo(const Event &ev);
    State state_resettingServo(const Event &ev);

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
        status.cutter_status = cutters.getStatus();
        status.servo_position =
            ejection_servo.getPosition(DeploymentConfigs::SERVO_CHANNEL);

        logger.log(status);
        StackLogger::getInstance()->updateStack(THID_DPL_FSM);
    }

    void initServo();
    void resetServo();
    void ejectNosecone();
    void disableServo();

    /**
     * Defer an event to be processed when the state machine goes back to
     * state_idle
     *
     * @param ev The event to be defered.
     */
    void deferEvent(const Event &ev);

    Cutter &cutters;
    Servo &ejection_servo;

    DeploymentStatus status;

    LoggerService &logger = *(LoggerService::getInstance());

    CircularBuffer<Event, DEFERRED_EVENTS_QUEUE_SIZE> deferred_events;

    uint8_t ejection_retry_count = 0;

    uint16_t ev_open_timeout_id  = 0;
    uint16_t ev_reset_timeout_id = 0;
    uint16_t ev_cut_timeout_id   = 0;
};

}  // namespace DeathStackBoard