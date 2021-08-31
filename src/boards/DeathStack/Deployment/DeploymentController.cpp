/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Deployment/DeploymentController.h>
#include <LoggerService/LoggerService.h>
#include <System/StackLogger.h>
#include <TimestampTimer.h>
#include <configs/DeploymentConfig.h>
#include <events/EventBroker.h>
#include <events/Events.h>

#include <LoggerService/LoggerService.h>
#include <System/StackLogger.h>
#include <TimestampTimer.h>
#include <configs/DeploymentConfig.h>
#include <events/EventBroker.h>
#include <events/Events.h>

namespace DeathStackBoard
{

DeploymentController::DeploymentController(HBridge* primaryCutter,
                                           HBridge* backupCutter,
                                           ServoInterface* ejection_servo)
    : FSM(&DeploymentController::state_initialization),
      primaryCutter{primaryCutter}, backupCutter{backupCutter},
      ejection_servo{ejection_servo}
{
    sEventBroker->subscribe(this, TOPIC_DPL);
}

DeploymentController::~DeploymentController()
{
    sEventBroker->unsubscribe(this);
}

void DeploymentController::logStatus(DeploymentControllerState current_state)
{
    status.timestamp            = TimestampTimer::getTimestamp();
    status.state                = current_state;
    status.servo_position       = ejection_servo->getCurrentPosition();
    status.primary_cutter_state = primaryCutter->getStatus();
    status.backup_cutter_state  = backupCutter->getStatus();

    LoggerService::getInstance()->log(status);
    StackLogger::getInstance()->updateStack(THID_DPL_FSM);
}

void DeploymentController::state_initialization(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(DeploymentControllerState::INITIALIZATION);

            LOG_DEBUG(log, "Entering state initialization");

            ejection_servo->enable();
            ejection_servo->reset();

            transition(&DeploymentController::state_idle);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state initialization");
            break;
        }

        default:
        {
            break;
        }
    }
}

void DeploymentController::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(DeploymentControllerState::IDLE);

            LOG_DEBUG(log, "Entering state idle");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state idle");
            break;
        }
        case EV_RESET_SERVO:
        {
            LOG_DEBUG(log, "Reset servo ");
            ejection_servo->reset();
            break;
        }
        case EV_WIGGLE_SERVO:
        {
            LOG_DEBUG(log, "Wiggle servo ");
            ejection_servo->selfTest();
            break;
        }
        case EV_NC_OPEN:
        {
            LOG_DEBUG(log, "Nosecone open event ");
            transition(&DeploymentController::state_noseconeEjection);
            break;
        }
        case EV_CUT_DROGUE:
        {
            LOG_DEBUG(log, "Cut drogue event ");
            transition(&DeploymentController::state_cuttingPrimary);
            break;
        }
        case EV_TEST_CUT_PRIMARY:
        {
            transition(&DeploymentController::state_testCuttingPrimary);
            break;
        }
        case EV_TEST_CUT_BACKUP:
        {
            transition(&DeploymentController::state_testCuttingBackup);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_noseconeEjection(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state nosecone_ejection");

            ejection_servo->set(DPL_SERVO_EJECT_POS);

            ev_nc_open_timeout_id = sEventBroker->postDelayed<NC_OPEN_TIMEOUT>(
                Event{EV_NC_OPEN_TIMEOUT}, TOPIC_DPL);

            logStatus(DeploymentControllerState::NOSECONE_EJECTION);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state nosecone_ejection");
            break;
        }
        case EV_NC_DETACHED:
        {
            LOG_DEBUG(log, "Nosecone detached event ");
            sEventBroker->removeDelayed(ev_nc_open_timeout_id);
            transition(&DeploymentController::state_idle);
            break;
        }
        case EV_NC_OPEN_TIMEOUT:
        {
            LOG_DEBUG(log, "Nosecone opening timeout ");
            transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_cuttingPrimary(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state cutting_primary");

            primaryCutter->enable();

            ev_nc_cutting_timeout_id = sEventBroker->postDelayed<CUT_DURATION>(
                Event{EV_CUTTING_TIMEOUT}, TOPIC_DPL);

            logStatus(DeploymentControllerState::CUTTING_PRIMARY);

            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state cutting_primary");

            primaryCutter->disable();

            break;
        }
        case EV_CUTTING_TIMEOUT:
        {
            LOG_DEBUG(log, "Primary cutter timeout ");
            transition(&DeploymentController::state_cuttingBackup);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_cuttingBackup(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state cutting_backup");

            backupCutter->enable();

            ev_nc_cutting_timeout_id = sEventBroker->postDelayed<CUT_DURATION>(
                Event{EV_CUTTING_TIMEOUT}, TOPIC_DPL);

            logStatus(DeploymentControllerState::CUTTING_BACKUP);

            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state cutting_backup");

            backupCutter->disable();

            break;
        }
        case EV_CUTTING_TIMEOUT:
        {
            LOG_DEBUG(log, "Backup cutter timeout ");
            transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_testCuttingPrimary(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            LOG_DEBUG(log, "Entering state test_cutting_primary");

            primaryCutter->enableTest(CUTTER_TEST_PWM_DUTY_CYCLE);

            ev_nc_cutting_timeout_id =
                sEventBroker->postDelayed<CUT_TEST_DURATION>(
                    Event{EV_CUTTING_TIMEOUT}, TOPIC_DPL);

            logStatus(DeploymentControllerState::TEST_CUTTING_PRIMARY);
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(log, "Exiting state test_cutting_primary");

            primaryCutter->disable();

            break;
        }
        case EV_CUTTING_TIMEOUT:
        {
            LOG_DEBUG(log, "Test primary cutter timeout ");
            transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_testCuttingBackup(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            backupCutter->enableTest(CUTTER_TEST_PWM_DUTY_CYCLE);

            ev_nc_cutting_timeout_id =
                sEventBroker->postDelayed<CUT_TEST_DURATION>(
                    Event{EV_CUTTING_TIMEOUT}, TOPIC_DPL);

            logStatus(DeploymentControllerState::TEST_CUTTING_BACKUP);

            LOG_DEBUG(log, "Entering state test_cutting_backup");
            break;
        }
        case EV_EXIT:
        {
            backupCutter->disable();

            LOG_DEBUG(log, "Exiting state test_cutting_backup");
            break;
        }
        case EV_CUTTING_TIMEOUT:
        {
            LOG_DEBUG(log, "Test backup cutter timeout ");
            transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

}  // namespace DeathStackBoard