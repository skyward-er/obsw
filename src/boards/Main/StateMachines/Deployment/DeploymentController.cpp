/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include "DeploymentController.h"

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/ActuatorsConfigs.h>
#include <Main/Radio/Radio.h>
#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <logger/Logger.h>
#include <miosix.h>

#include "DeploymentConfig.h"

using namespace Boardcore;
using namespace Main::DeploymentConfig;
using namespace Main::ActuatorsConfigs;

namespace Main
{

DeploymentController::DeploymentController()
    : FSM(&DeploymentController::state_init)
{
    memset(&status, 0, sizeof(DeploymentControllerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

DeploymentController::~DeploymentController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void DeploymentController::state_init(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                                   DPL_SERVO_EJECT_POS);
            Actuators::getInstance().enableServo(EXPULSION_SERVO);

            transition(&DeploymentController::state_idle);

            logStatus(INIT);
            LOG_DEBUG(logger, "[Deployment] entering state init\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[Deployment] exiting state init\n");
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
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(IDLE);
            LOG_DEBUG(logger, "[Deployment] entering state idle\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[Deployment] exiting state idle\n");
            break;
        }
        case DPL_WIGGLE:
        {
            wiggle_servo();
            break;
        }
        case DPL_OPEN:
        {
            Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                                   DPL_SERVO_EJECT_POS);
            break;
        }
        case DPL_RESET:
        {
            Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                                   DPL_SERVO_EJECT_POS);
            break;
        }
        case DPL_OPEN_NC:
        {
            transition(&DeploymentController::state_nosecone_ejection);
            break;
        }
        case DPL_CUT_DROGUE:
        {
            transition(&DeploymentController::state_cutting);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_nosecone_ejection(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                                   DPL_SERVO_EJECT_POS);

            open_nc_timeout_event_id =
                EventBroker::getInstance().postDelayed<OPEN_NC_TIMEOUT>(
                    Boardcore::Event{DPL_OPEN_NC_TIMEOUT}, TOPIC_DPL);

            logStatus(NOSECONE_EJECTION);
            LOG_DEBUG(logger,
                      "[Deployment] entering state nosecone_ejection\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[Deployment] exiting state nosecone_ejection\n");
            break;
        }
        case DPL_OPEN_NC_TIMEOUT:
        {
            transition(&DeploymentController::state_idle);
            break;
        }
        case FLIGHT_NC_DETACHED:
        {
            EventBroker::getInstance().removeDelayed(open_nc_timeout_event_id);
            transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::state_cutting(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            start_cutting();

            nc_cutting_timeout_event_id =
                EventBroker::getInstance().postDelayed<CUT_DURATION>(
                    Boardcore::Event{DPL_CUT_TIMEOUT}, TOPIC_DPL);

            logStatus(CUTTING);
            LOG_DEBUG(logger, "[Deployment] entering state cutting\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[Deployment] exiting state cutting\n");
            break;
        }
        case DPL_CUT_TIMEOUT:
        {
            stop_cutting();
            transition(&DeploymentController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void DeploymentController::wiggle_servo()
{
    for (int i = 0; i < 2; i++)
    {
        Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                               DPL_SERVO_EJECT_POS);
        miosix::Thread::sleep(500);
        Actuators::getInstance().setServoAngle(EXPULSION_SERVO,
                                               DPL_SERVO_RESET_POS);
        miosix::Thread::sleep(500);
    }
}

void DeploymentController::start_cutting()
{
    // TODO: Change with actual cutter
    Actuators::getInstance().led1.high();
}

void DeploymentController::stop_cutting()
{
    // TODO: Change with actual cutter
    Actuators::getInstance().led1.low();
}

void DeploymentController::logStatus(DeploymentControllerState state)
{
    status.timestamp = TimestampTimer::getInstance().getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

}  // namespace Main
