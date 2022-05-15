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

#include "NASController.h"

#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;

namespace Main
{

NASController::NASController() : FSM(&NASController::state_idle)
{
    memset(&status, 0, sizeof(NASControllerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

NASController::~NASController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void NASController::state_idle(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(IDLE);
            LOG_DEBUG(logger, "[NAS] entering state idle\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[NAS] exiting state idle\n");
            break;
        }
        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }
        default:
        {
            break;
        }
    }
}

void NASController::state_calibrating(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            calibrate();

            logStatus(CALIBRATING);
            LOG_DEBUG(logger, "[NAS] entering state calibrating\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[NAS] exiting state calibrating\n");
            break;
        }
        case NAS_READY:
        {
            transition(&NASController::state_ready);
            break;
        }
        default:
        {
            break;
        }
    }
}

void NASController::state_ready(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(READY);
            LOG_DEBUG(logger, "[NAS] entering state ready\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[NAS] exiting state ready\n");
            break;
        }
        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            transition(&NASController::state_active);
            break;
        }
        default:
        {
            break;
        }
    }
}

void NASController::state_active(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(ACTIVE);
            LOG_DEBUG(logger, "[NAS] entering state active\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[NAS] exiting state active\n");
            break;
        }
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&NASController::state_end);
            break;
        }
        default:
        {
            break;
        }
    }
}

void NASController::state_end(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            logStatus(END);
            LOG_DEBUG(logger, "[NAS] entering state end\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[NAS] exiting state end\n");
            break;
        }

        default:
        {
            break;
        }
    }
}

void NASController::calibrate()
{
    // ...
}

void NASController::logStatus(NASControllerState state)
{
    status.timestamp = TimestampTimer::getInstance().getTimestamp();
    status.state     = state;

    Logger::getInstance().log(state);
}

}  // namespace Main
