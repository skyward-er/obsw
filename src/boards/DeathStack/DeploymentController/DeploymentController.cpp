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

#include "DeathStack/configs/DeploymentConfig.h"
#include "DeathStack/events/Events.h"
#include "DeploymentController.h"

#include "Motor/MotorDriver.h"
#include "events/EventBroker.h"

namespace DeathStackBoard
{

using namespace DeploymentConfigs;

DeploymentController::DeploymentController(Cutter& cutter,
                                           Servo& ejection_servo)
    : HSM(&DeploymentController::state_initialization, 4096, 2),
      cutters(cutter), ejection_servo(ejection_servo)
{
    memset(&status, 0, sizeof(DeploymentStatus));

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
    initServo();

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
            logStatus(DeploymentCTRLState::DPL_IDLE);
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
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            TRACE("[DPL_CTRL] state_idle EXIT\n");

            break;
        }
        case EV_NC_OPEN:
        {
            retState =
                transition(&DeploymentController::state_ejectingNosecone);
            break;
        }
        case EV_CUT_DROGUE:
        {
            cut_backup = true;
            retState = transition(&DeploymentController::state_cuttingPrimary);
            break;
        }
        case EV_CUT_PRIMARY:
        {
            cut_backup = false;
            retState = transition(&DeploymentController::state_cuttingPrimary);
            break;
        }
        case EV_CUT_BACKUP:
        {
            retState = transition(&DeploymentController::state_cuttingBackup);
            break;
        }
        case EV_TEST_CUTTER_PRIMARY:
        {
            retState = transition(&DeploymentController::state_testingPrimary);
            break;
        }
        case EV_TEST_CUTTER_BACKUP:
        {
            retState = transition(&DeploymentController::state_testingBackup);
            break;
        }
        case EV_RESET_SERVO:
        {
            resetServo();
            break;
        }
        case EV_WIGGLE_SERVO
        {
            wiggleServo();
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

State DeploymentController::state_ejectingNosecone(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            ejectNosecone();
            ev_open_timeout_id = sEventBroker->postDelayed<NC_OPEN_TIMEOUT>(
                Event{EV_TIMEOUT_NC_OPEN}, TOPIC_DEPLOYMENT);

            logStatus(DeploymentCTRLState::EJECTING_NC);
            TRACE("[DPL_CTRL] state_ejectingNosecone ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            TRACE("[DPL_CTRL] state_ejectingNosecone INIT\n");
            break;
        }
        case EV_EXIT:
        {
            // disableServo();
            sEventBroker->removeDelayed(ev_open_timeout_id);

            TRACE("[DPL_CTRL] state_openingNosecone EXIT\n");

            break;
        }
        case EV_CUT_DROGUE:
        {
            deferred_events.put(ev);
            break;
        }
        case EV_NC_DETACHED:
        case EV_TIMEOUT_NC_OPEN:
        {
            retState = transition(&DeploymentController::state_idle);
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

State DeploymentController::state_cuttingPrimary(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting drogue
            cutters.enablePrimaryCutter();

            logStatus(DeploymentCTRLState::CUTTING_PRIMARY);

            ev_cut_timeout_id = sEventBroker->postDelayed<CUT_DURATION>(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT);

            TRACE("[DPL_CTRL] state_cuttingPrimary ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutters.disablePrimaryCutter();

            sEventBroker->removeDelayed(ev_cut_timeout_id);
            TRACE("[DPL_CTRL] state_cuttingPrimary EXIT\n");

            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            if (cut_backup)
            {
                retState =
                    transition(&DeploymentController::state_cuttingBackup);
            }
            else
            {
                retState = transition(&DeploymentController::state_idle);
            }
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

State DeploymentController::state_cuttingBackup(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting rogallina
            cutters.enableBackupCutter();

            logStatus(DeploymentCTRLState::CUTTING_BACKUP);

            ev_cut_timeout_id = sEventBroker->postDelayed<CUT_DURATION>(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT);

            TRACE("[DPL_CTRL] state_cuttingBackup ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutters.disableBackupCutter();

            sEventBroker->removeDelayed(ev_cut_timeout_id);

            TRACE("[DPL_CTRL] state_cuttingBackup EXIT\n");

            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            retState = transition(&DeploymentController::state_idle);
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

State DeploymentController::state_testingPrimary(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            cutters.enableTestPrimaryCutter();

            logStatus(DeploymentCTRLState::TESTING_PRIMARY);

            ev_cut_timeout_id = sEventBroker->postDelayed<CUT_TEST_DURATION>(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT);

            TRACE("[DPL_CTRL] state_testingPrimary ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutters.disablePrimaryCutter();

            sEventBroker->removeDelayed(ev_cut_timeout_id);

            TRACE("[DPL_CTRL] state_testingPrimary EXIT\n");

            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            retState = transition(&DeploymentController::state_idle);
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

State DeploymentController::state_testingBackup(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            // Cutting rogallina
            cutters.enableTestBackupCutter();

            logStatus(DeploymentCTRLState::TESTING_BACKUP);

            ev_cut_timeout_id = sEventBroker->postDelayed<CUT_TEST_DURATION>(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT);

            TRACE("[DPL_CTRL] state_testingBackup ENTRY\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            cutters.disableBackupCutter();

            sEventBroker->removeDelayed(ev_cut_timeout_id);

            TRACE("[DPL_CTRL] state_testingBackup EXIT\n");

            break;
        }
        case EV_TIMEOUT_CUTTING:
        {
            retState = transition(&DeploymentController::state_idle);
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

void DeploymentController::initServo()
{
    ejection_servo.enable(SERVO_CHANNEL);

    ejection_servo.setPosition(SERVO_CHANNEL, SERVO_RESET_POS);

    ejection_servo.start();
}

void DeploymentController::resetServo()
{
    ejection_servo.setPosition(SERVO_CHANNEL, SERVO_RESET_POS);
}

void DeploymentController::ejectNosecone()
{
    ejection_servo.setPosition(SERVO_CHANNEL, SERVO_EJECT_POS);
}

void DeploymentController::disableServo()
{
    // Do not reset the position to 0

    ejection_servo.stop();
    ejection_servo.disable(SERVO_CHANNEL);
}

void DeploymentController::wiggleServo()
{
    for (int i = 0; i < 3; i++)
    {
        ejection_servo.setPosition(SERVO_CHANNEL, SERVO_RESET_POS + SERVO_WIGGLE_AMPLITUDE);
        Thread::sleep(500);
        ejection_servo.setPosition(SERVO_CHANNEL, SERVO_RESET_POS - SERVO_WIGGLE_AMPLITUDE);
        Thread::sleep(500);
        ejection_servo.setPosition(SERVO_CHANNEL, SERVO_RESET_POS);
        Thread::sleep(500);
    }
}

}  // namespace DeathStackBoard
