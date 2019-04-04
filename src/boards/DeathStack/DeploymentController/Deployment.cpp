/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alvise de' Faveri Tron
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

#include <stdexcept>

#include "DeathStack/Events.h"
#include "DeathStack/configs/DeploymentConfig.h"
#include "Deployment.h"
#include "Motor/MotorDriver.h"
#include "PinObserver.h"
#include "events/EventBroker.h"

namespace DeathStackBoard
{

DeploymentController::DeploymentController()
    : HSM(&DeploymentController::state_initialization), motor()
{
    sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
}

DeploymentController::~DeploymentController()
{
    sEventBroker->unsubscribe(this);
}

State DeploymentController::state_initialization(const Event& ev)
{
    // Nothing to do during initialization

    UNUSED(ev);
    return transition(&DeploymentController::state_idle);
}

State DeploymentController::state_idle(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Process deferred events
            try
            {
                while (!deferred_events.isEmpty())
                {
                    postEvent(deferred_events.pop());
                }
            }
            catch (...)
            {
                TRACE("Tried to pop empty circularbuffer!\n");
            }

            TRACE("[DPL_CTRL] state_idle ENTRY\n");
            logStatus(DeploymentCTRLState::DPL_IDLE);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_NC_OPEN:
        {
            retState = transition(&DeploymentController::state_openingNosecone);
            break;
        }
        case EV_CUT_MAIN:
        {
            retState = transition(&DeploymentController::state_cuttingMain);
            break;
        }
        case EV_CUT_DROGUE:
        {
            retState = transition(&DeploymentController::state_cuttingDrogue);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_openingNosecone(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Start the motor to open the nosecone
            motor.start(MOTOR_OPEN_DIR, MOTOR_OPEN_DUTY_CYCLE);

            ev_open_timeout_id = sEventBroker->postDelayed(
                Event{EV_TIMEOUT_MOT_OPEN}, TOPIC_DEPLOYMENT, NC_OPEN_TIMEOUT);

            TRACE("[DPL_CTRL] state_openingNosecone ENTRY\n");
            logStatus(DeploymentCTRLState::OPENING_NC);
            break;
        }
        case EV_INIT:
        {
            retState = transition(&DeploymentController::state_spinning);
            break;
        }
        case EV_EXIT:
        {
            // Stop the motor
            motor.stop();

            sEventBroker->removeDelayed(ev_min_open_time_id);
            sEventBroker->removeDelayed(ev_open_timeout_id);
            break;
        }
        case EV_TIMEOUT_MOT_OPEN:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        case EV_CUT_DROGUE:
        {
            deferred_events.put(ev);
            break;
        }
        case EV_CUT_MAIN:
        {
            deferred_events.put(ev);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_spinning(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            ev_min_open_time_id = sEventBroker->postDelayed(
                Event{EV_MOT_MIN_OPEN_TIME}, TOPIC_DEPLOYMENT,
                NC_MINIMUM_OPENING_TIME);

            TRACE("[DPL_CTRL] state_spinning ENTRY\n");
            logStatus(DeploymentCTRLState::SPINNING);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_NC_DETACHED:
        {
            retState =
                transition(&DeploymentController::state_awaitingOpenTime);
            break;
        }
        case EV_MOT_MIN_OPEN_TIME:
        {
            retState =
                transition(&DeploymentController::state_awaitingDetachment);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::state_openingNosecone);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_awaitingOpenTime(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[DPL_CTRL] state_awaitingOpenTime ENTRY\n");
            logStatus(DeploymentCTRLState::AWAITING_MINOPENTIME);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_MOT_MIN_OPEN_TIME:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::state_openingNosecone);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_awaitingDetachment(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[DPL_CTRL] state_awaitingDetachment ENTRY\n");
            logStatus(DeploymentCTRLState::AWAITING_DETACHMENT);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_NC_DETACHED:
        {
            retState = transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            retState = tran_super(&DeploymentController::state_openingNosecone);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_cuttingDrogue(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting drogue
            cutter.startCutDrogue();

            ev_cut_timeout_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT,
                MAXIMUM_CUTTING_DURATION);

            TRACE("[DPL_CTRL] state_cuttingDrogue ENTRY\n");
            logStatus(DeploymentCTRLState::CUTTING_DROGUE);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutter.stopCutDrogue();

            sEventBroker->removeDelayed(ev_cut_timeout_id);
            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            transition(&DeploymentController::state_idle);
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

State DeploymentController::state_cuttingMain(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting rogallina
            cutter.startCutMainChute();

            ev_cut_timeout_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT,
                MAXIMUM_CUTTING_DURATION);

            TRACE("[DPL_CTRL] state_cuttingMain ENTRY\n");
            logStatus(DeploymentCTRLState::CUTTING_MAIN);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutter.stopCutMainChute();

            sEventBroker->removeDelayed(ev_cut_timeout_id);
            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            transition(&DeploymentController::state_idle);
        }
        default:
        {
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
        }
    }
    return retState;
}

}  // namespace DeathStackBoard
