/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include "FSRController.h"

#include <MainComputer/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <logger/Logger.h>
#include <miosix.h>

#include "FSRConfig.h"

using namespace Boardcore;
using namespace MainComputer::FSRConfig;

namespace MainComputer
{

FSRController::FSRController() : FSM(&FSRController::state_idle)
{
    memset(&status, 0, sizeof(FSRControllerStatus));
    sEventBroker.subscribe(this, TOPIC_FLIGHT);
    sEventBroker.subscribe(this, TOPIC_FSR);
}

FSRController::~FSRController() { sEventBroker.unsubscribe(this); }

void FSRController::state_idle(const Event& ev)
{
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            logStatus(IDLE);
            LOG_DEBUG(logger, "[FSR] entering state idle\n");
            break;
        }
        case EV_EXIT:
        {
            LOG_DEBUG(logger, "[FSR] exiting state idle\n");
            break;
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            transition(&FSRController::state_liftoff);
            break;
        }
        case FLIGHT_DPL_ALT_DETECTED:
        {
            transition(&FSRController::state_main_deployment);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FSRController::state_liftoff(const Event& ev)
{
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            sEventBroker.postDelayed<LIFTOFF_STATS_TIMEOUT>(
                Boardcore::Event{FSR_STATS_TIMEOUT}, TOPIC_FSR);

            logStatus(LIFTOFF);
            LOG_DEBUG(logger, "[FSR] entering state liftoff\n");
            break;
        }
        case EV_EXIT:
        {
            log_liftoff_stats();

            LOG_DEBUG(logger, "[FSR] exiting state liftoff\n");
            break;
        }
        case FSR_STATS_TIMEOUT:
        {
            transition(&FSRController::state_ascending);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FSRController::state_ascending(const Event& ev)
{
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            logStatus(ASCENDING);
            LOG_DEBUG(logger, "[FSR] entering state ascending\n");
            break;
        }
        case EV_EXIT:
        {
            log_liftoff_stats();
            log_apogee_stats();

            LOG_DEBUG(logger, "[FSR] exiting state ascending\n");
            break;
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            sEventBroker.postDelayed<APOGEE_STATS_TIMEOUT>(
                Boardcore::Event{FSR_STATS_TIMEOUT}, TOPIC_FSR);
            break;
        }
        case FSR_STATS_TIMEOUT:
        {
            transition(&FSRController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FSRController::state_main_deployment(const Event& ev)
{
    switch (ev.code)
    {
        case EV_ENTRY:
        {
            sEventBroker.postDelayed<MAIN_DPL_STATS_TIMEOUT>(
                Boardcore::Event{FSR_STATS_TIMEOUT}, TOPIC_FSR);

            logStatus(MAIN_DEPLOYMENT);
            LOG_DEBUG(logger, "[FSR] entering state main_deployment\n");
            break;
        }
        case EV_EXIT:
        {
            log_main_dpl_stats();

            LOG_DEBUG(logger, "[FSR] exiting state main_deployment\n");
            break;
        }
        case FSR_STATS_TIMEOUT:
        {
            transition(&FSRController::state_idle);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FSRController::log_apogee_stats()
{
    // ...
}

void FSRController::log_liftoff_stats()
{
    // ...
}

void FSRController::log_main_dpl_stats()
{
    // ...
}

void FSRController::logStatus(FSRControllerState state)
{
    status.timestamp = TimestampTimer::getInstance().getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

}  // namespace MainComputer
