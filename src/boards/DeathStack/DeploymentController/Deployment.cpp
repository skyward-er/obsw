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

#include "Deployment.h"
#include "DeathStack/Events.h"
#include "DeathStack/configs/DeploymentConfig.h"
#include "events/EventBroker.h"
#include "Motor/MotorDriver.h"
#include "PinObserver.h"

namespace DeathStackBoard
{
DeploymentController::DeploymentController(PinObserver* pin_observer)
    : FSM(&DeploymentController::state_idle), motor(pin_observer)
{
    sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
}

DeploymentController::~DeploymentController() {}

void DeploymentController::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("[DPL_CTRL] state_idle ENTRY\n");
            logStatus(DeploymentCTRLState::IDLE);
            break;
        case EV_EXIT:
            break;
        case EV_NC_OPEN:
        case EV_TC_NC_OPEN:
        {
            transition(&DeploymentController::state_openingNosecone);
            break;
        }
        case EV_TC_NC_CLOSE:
        {
            transition(&DeploymentController::state_closingNosecone);
            break;
        }
        case EV_TC_CUT_MAIN:
            transition(&DeploymentController::state_cuttingMain);
            break;
        case EV_TC_CUT_ALL:
        {
            TRACE("[DPL_CTRL] state_idle EV_TC_CUT_ALL\n");
            cut_main = true;
            // Continue below. No break!
        }
        case EV_TC_CUT_FIRST_DROGUE:
        case EV_CUT_DROGUE:
        {
            transition(&DeploymentController::state_cuttingDrogue);
            break;
        }
        default:
            break;
    }
}

void DeploymentController::state_openingNosecone(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[DPL_CTRL] state_openingNosecone ENTRY\n");
            //Start the motor
            motor.start(MOTOR_OPEN_DIR, MOTOR_OPEN_DUTY_CYCLE);

            min_open_time_elapsed = false;
            nc_detached           = false;

            delayed_ev_id_1 = sEventBroker->postDelayed(
                Event{EV_MOT_MIN_OPEN_TIME}, TOPIC_DEPLOYMENT,
                NC_MINIMUM_OPENING_TIME);

            delayed_ev_id_2 = sEventBroker->postDelayed(
                Event{EV_TIMEOUT_MOT_OPEN}, TOPIC_DEPLOYMENT, NC_OPEN_TIMEOUT);
            
            logStatus(DeploymentCTRLState::OPENING_NC);
            break;
        }
        case EV_EXIT:
        {
            // Stop the motor
            motor.stop();

            sEventBroker->removeDelayed(delayed_ev_id_1);
            sEventBroker->removeDelayed(delayed_ev_id_2);
            break;
        }
        case EV_MOT_MIN_OPEN_TIME:
        {
            if (!nc_detached)
            {
                TRACE(
                    "[DPL_CTRL] state_openingNosecone EV_MOT_MIN_OPEN_TIME "
                    "elapsed.\n");
                // We can now stop the motor when the nosecone detachment is
                // detected.
                min_open_time_elapsed = true;
            }
            else
            {
                // nc detachment already detected. stop the motor.
                transition(&DeploymentController::state_idle);
            }
            break;
        }
        case EV_NC_DETACHED:
        {
            if (!min_open_time_elapsed)
            {
                TRACE(
                    "[DPL_CTRL] state_openingNosecone EV_NC_DETACHED before "
                    "min open time.\n");
                // Minimum opening time not yet elapsed. Continue spinning the
                // motor, but signal that a nosecone detachment has been
                // detected.
                nc_detached = true;
            }
            else
            {
                // Minimum opening time elapsed, we can stop opening
                transition(&DeploymentController::state_idle);
            }
            break;
        }
        case EV_TC_NC_STOP:
        case EV_TIMEOUT_MOT_OPEN:
        {
            // Stop the motor unconditionally.
            transition(&DeploymentController::state_idle);
            break;
        }
    }
}

void DeploymentController::state_closingNosecone(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {           
            TRACE("[DPL_CTRL] state_closingNosecone ENTRY\n");

            motor.start(MOTOR_CLOSE_DIR, MOTOR_CLOSE_DUTY_CYCLE);

            delayed_ev_id_1 = sEventBroker->postDelayed(
                Event{EV_TIMEOUT_MOT_CLOSE}, TOPIC_DEPLOYMENT, NC_CLOSE_TIMEOUT);
            
            logStatus(DeploymentCTRLState::CLOSING_NC);
            break;
        }
        case EV_EXIT:
        {
            motor.stop();
            
            sEventBroker->removeDelayed(delayed_ev_id_1);
            break;
        }
        case EV_TC_NC_STOP:
        case EV_TIMEOUT_MOT_CLOSE:
        {
            // stop the motor unconditionally.
            transition(&DeploymentController::state_idle);
            break;
        }
    }
}

void DeploymentController::state_cuttingDrogue(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("[DPL_CTRL] state_cuttingDrogue ENTRY\n");
            cutter.startCutDrogue();
            delayed_ev_id_1 = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT,
                MAXIMUM_CUTTING_DURATION);
            
            logStatus(DeploymentCTRLState::CUTTING_DROGUE);
            break;
        case EV_EXIT:
            cutter.stopCutDrogue();

            sEventBroker->removeDelayed(delayed_ev_id_1);
            break;
        case EV_TC_CUT_MAIN:
        case EV_TC_CUT_ALL:
            cut_main = true;
            break;
        case EV_TIMEOUT_CUTTING:
            TRACE("[DPL_CTRL] state_cuttingDrogue TIMEOUT\n");
            if (!cut_main)
            {
                transition(&DeploymentController::state_idle);
            }
            else
            {
                transition(&DeploymentController::state_cuttingMain);
            }
            break;
        default:
            break;
    }
}

void DeploymentController::state_cuttingMain(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("[DPL_CTRL] state_cuttingMain ENTRY\n");
            
            cut_main = false;
            cutter.startCutMainChute();
            
            delayed_ev_id_1 = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT,
                MAXIMUM_CUTTING_DURATION);
            
            logStatus(DeploymentCTRLState::CUTTING_MAIN);
            break;
        case EV_EXIT:
            cutter.stopCutMainChute();

            sEventBroker->removeDelayed(delayed_ev_id_1);
            break;
        case EV_TIMEOUT_CUTTING:
            TRACE("[DPL_CTRL] state_cuttingMain TIMEOUT\n");
            transition(&DeploymentController::state_idle);
            break;
        default:
            break;
    }
}

}  // namespace DeathStackBoard
