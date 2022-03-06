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

#include <MainComputer/Actuators/Actuators.h>
#include <MainComputer/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include "AirBrakesConfig.h"

using namespace Boardcore;
using namespace MainComputer::AirBrakesConfigs;

namespace MainComputer
{

AirBrakesController::AirBrakesController()
    : FSM(&AirBrakesController::state_init)
{
    memset(&status, 0, sizeof(AirBrakesControllerStatus));
    sEventBroker.subscribe(this, TOPIC_ABK);
    sEventBroker.subscribe(this, TOPIC_FLIGHT);
}

AirBrakesController::~AirBrakesController() { sEventBroker.unsubscribe(this); }

void AirBrakesController::state_init(const Event& ev)
{
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            Actuators::getInstance().servoAirbrakes.setPosition120Deg(
                ABK_SERVO_MIN_POS);
            Actuators::getInstance().servoAirbrakes.enable();

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
    switch (ev.code)
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
            Actuators::getInstance().servoExpulsion.setPosition120Deg(
                ABK_SERVO_MAX_POS);
            break;
        }
        case ABK_RESET:
        {

            Actuators::getInstance().servoExpulsion.setPosition120Deg(
                ABK_SERVO_MIN_POS);
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
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            shadow_mode_timeout_event_id =
                sEventBroker.postDelayed<SHADOW_MODE_TIMEOUT>(
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
    switch (ev.code)
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
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            // TODO: algorithm.end()

            Actuators::getInstance().servoAirbrakes.setPosition120Deg(
                ABK_SERVO_MIN_POS);

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
        Actuators::getInstance().servoAirbrakes.setPosition120Deg(
            ABK_SERVO_MIN_POS + ABK_SERVO_WIGGLE_AMPLITUDE);
        miosix::Thread::sleep(500);
        Actuators::getInstance().servoAirbrakes.setPosition120Deg(
            ABK_SERVO_MIN_POS);
        miosix::Thread::sleep(500);
    }
}

void AirBrakesController::logStatus(AirBrakesControllerState state)
{
    status.timestamp = TimestampTimer::getInstance().getTimestamp();
    status.state     = state;

    Logger::getInstance().log(state);
}

}  // namespace MainComputer
