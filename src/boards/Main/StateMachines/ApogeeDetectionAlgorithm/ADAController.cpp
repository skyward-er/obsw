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

#include "ADAController.h"

#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>

#include "ADAConfig.h"

using namespace Boardcore;
using namespace Main::ADAConfig;

namespace Main
{

ADAController::ADAController() : FSM(&ADAController::state_idle)
{
    memset(&status, 0, sizeof(ADAControllerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

ADAController::~ADAController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void ADAController::state_idle(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(IDLE);
            LOG_DEBUG(logger, "[ADA] entering state idle\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state idle\n");
            break;
        }
        case ADA_CALIBRATE:
        {
            transition(&ADAController::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_calibrating(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            calibrate();

            logStatus(CALIBRATING);
            LOG_DEBUG(logger, "[ADA] entering state calibrating\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state calibrating\n");
            break;
        }
        case ADA_READY:
        {
            transition(&ADAController::state_ready);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_ready(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            // ...

            logStatus(READY);

            LOG_DEBUG(logger, "[ADA] entering state ready\n");
            break;
        }
        case EV_EXIT:
        {
            // ...

            LOG_DEBUG(logger, "[ADA] exiting state ready\n");
            break;
        }
        case ADA_CALIBRATE:
        {
            transition(&ADAController::state_calibrating);
            break;
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            transition(&ADAController::state_shadow_mode);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_shadow_mode(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            shadow_mode_timeout_event_id =
                EventBroker::getInstance().postDelayed<SHADOW_MODE_TIMEOUT>(
                    Boardcore::Event{ADA_SHADOW_MODE_TIMEOUT}, TOPIC_ABK);

            logStatus(SHADOW_MODE);
            LOG_DEBUG(logger, "[ADA] entering state shadow_mode\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state shadow_mode\n");
            break;
        }
        case ADA_SHADOW_MODE_TIMEOUT:
        {
            transition(&ADAController::state_active);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_active(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(ACTIVE);
            LOG_DEBUG(logger, "[ADA] entering state active\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state active\n");
            break;
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            transition(&ADAController::state_pressure_stabilization);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_pressure_stabilization(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            press_stab_timeout_event_id =
                EventBroker::getInstance().postDelayed<PRES_STAB_TIMEOUT>(
                    Boardcore::Event{ADA_PRESS_STAB_TIMEOUT}, TOPIC_ADA);

            logStatus(PRESSURE_STABILIZATION);
            LOG_DEBUG(logger, "[ADA] entering state pressure_stabilization\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state pressure_stabilization\n");
            break;
        }
        case ADA_PRESS_STAB_TIMEOUT:
        {
            transition(&ADAController::state_drogue_descent);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_drogue_descent(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(DROGUE_DESCENT);
            LOG_DEBUG(logger, "[ADA] entering state drogue_descent\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state drogue_descent\n");
            break;
        }
        case FLIGHT_DPL_ALT_DETECTED:
        {
            transition(&ADAController::state_terminal_descent);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_terminal_descent(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(TERMINAL_DESCENT);
            LOG_DEBUG(logger, "[ADA] entering state terminal_descent\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state terminal_descent\n");
            break;
        }
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ADAController::state_landed);
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::state_landed(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(LANDED);
            LOG_DEBUG(logger, "[ADA] entering state landed\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[ADA] exiting state landed\n");
            break;
        }
        default:
        {
            break;
        }
    }
}

void ADAController::calibrate()
{
    // ...
}

void ADAController::logStatus(ADAControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

}  // namespace Main
