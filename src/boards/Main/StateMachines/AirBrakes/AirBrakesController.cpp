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

#include "AirBrakesController.h"

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/ActuatorsConfigs.h>
#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include "AirBrakesConfig.h"

using namespace Boardcore;
using namespace Main::AirBrakesConfigs;
using namespace Main::ActuatorsConfigs;

namespace Main
{

AirBrakesController::AirBrakesController()
    : FSM(&AirBrakesController::state_init)
{
    memset(&status, 0, sizeof(AirBrakesControllerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_ABK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

AirBrakesController::~AirBrakesController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void AirBrakesController::state_init(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                                   DPL_SERVO_RESET_POS);
            Actuators::getInstance().enableServo(AIRBRAKES_SERVO);

            transition(&AirBrakesController::state_idle);

            logStatus(INIT);
            LOG_DEBUG(logger, "[AirBrakes] entering state init\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[AirBrakes] exiting state init\n");
            break;
        }
        default:
        {
            break;
        }
    }
}

void AirBrakesController::state_idle(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(IDLE);
            LOG_DEBUG(logger, "[AirBrakes] entering state idle\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[AirBrakes] exiting state idle\n");
            break;
        }
        case ABK_WIGGLE:
        {
            wiggle_servo();
            break;
        }
        case ABK_OPEN:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                                   ABK_SERVO_ROTATION);
            break;
        }
        case ABK_RESET:
        {
            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);
            break;
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            transition(&AirBrakesController::state_shadow_mode);
            break;
        }
        default:
        {
            break;
        }
    }
}

void AirBrakesController::state_shadow_mode(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            shadow_mode_timeout_event_id =
                EventBroker::getInstance().postDelayed<SHADOW_MODE_TIMEOUT>(
                    Boardcore::Event{ABK_SHADOW_MODE_TIMEOUT}, TOPIC_ABK);

            logStatus(SHADOW_MODE);
            LOG_DEBUG(logger, "[AirBrakes] entering state shadow mode\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[AirBrakes] exiting state shadow mode\n");
            break;
        }
        case ABK_SHADOW_MODE_TIMEOUT:
        {
            transition(&AirBrakesController::state_active);
            break;
        }
        default:
        {
            break;
        }
    }
}

void AirBrakesController::state_active(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            // TODO: algorithm.begin()

            logStatus(ACTIVE);
            LOG_DEBUG(logger, "[AirBrakes] entering state active\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[AirBrakes] exiting state active\n");
            break;
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            transition(&AirBrakesController::state_end);
            break;
        }
        case ABK_DISABLE:
        {
            transition(&AirBrakesController::state_end);
            break;
        }
        default:
        {
            break;
        }
    }
}

void AirBrakesController::state_end(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            // TODO: algorithm.end()

            Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);

            logStatus(END);
            LOG_DEBUG(logger, "[AirBrakes] entering state end\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[AirBrakes] exiting state end\n");
            break;
        }
        default:
        {
            break;
        }
    }
}

void AirBrakesController::wiggle_servo()
{
    for (int i = 0; i < 2; i++)
    {
        Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO,
                                               ABK_SERVO_ROTATION);
        miosix::Thread::sleep(500);
        Actuators::getInstance().setServoAngle(AIRBRAKES_SERVO, 0);
        miosix::Thread::sleep(500);
    }
}

void AirBrakesController::logStatus(AirBrakesControllerState state)
{
    status.timestamp = TimestampTimer::getInstance().getTimestamp();
    status.state     = state;

    Logger::getInstance().log(state);
}

}  // namespace Main
