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

#include "FlightStatsRecorder.h"

#include <Main/Configs/FlightStatsRecorderConfig.h>
#include <Main/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <logger/Logger.h>
#include <miosix.h>

using namespace Boardcore;
using namespace Main::FlightStatsRecorderConfig;

namespace Main
{

FlightStatsRecorder::FlightStatsRecorder()
    : FSM(&FlightStatsRecorder::state_idle)
{
    memset(&status, 0, sizeof(FlightModeManagerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FSR);
}

FlightStatsRecorder::~FlightStatsRecorder()
{
    EventBroker::getInstance().unsubscribe(this);
}

FlightModeManagerStatus FlightStatsRecorder::getStatus() { return status; }

void FlightStatsRecorder::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(IDLE);
        }
        case FLIGHT_LIFTOFF_DETECTED:
        {
            return transition(&FlightStatsRecorder::state_liftoff);
        }
        case FLIGHT_DPL_ALT_DETECTED:
        {
            return transition(&FlightStatsRecorder::state_main_deployment);
        }
    }
}

void FlightStatsRecorder::state_liftoff(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(LIFTOFF);

            EventBroker::getInstance().postDelayed<LIFTOFF_STATS_TIMEOUT>(
                Boardcore::Event{FSR_STATS_TIMEOUT}, TOPIC_FSR);
            break;
        }
        case EV_EXIT:
        {
            return logLiftoffStats();
        }
        case FSR_STATS_TIMEOUT:
        {
            return transition(&FlightStatsRecorder::state_ascending);
        }
    }
}

void FlightStatsRecorder::state_ascending(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ASCENDING);
        }
        case EV_EXIT:
        {
            logLiftoffStats();
            logApogeeStats();
            break;
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            EventBroker::getInstance().postDelayed<APOGEE_STATS_TIMEOUT>(
                Boardcore::Event{FSR_STATS_TIMEOUT}, TOPIC_FSR);
            break;
        }
        case FSR_STATS_TIMEOUT:
        {
            return transition(&FlightStatsRecorder::state_idle);
        }
    }
}

void FlightStatsRecorder::state_main_deployment(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(MAIN_DEPLOYMENT);

            EventBroker::getInstance().postDelayed<MAIN_DPL_STATS_TIMEOUT>(
                Boardcore::Event{FSR_STATS_TIMEOUT}, TOPIC_FSR);
            break;
        }
        case EV_EXIT:
        {
            return logMainDplStats();
        }
        case FSR_STATS_TIMEOUT:
        {
            return transition(&FlightStatsRecorder::state_idle);
        }
    }
}

void FlightStatsRecorder::logStatus(FlightModeManagerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

void FlightStatsRecorder::logApogeeStats()
{
    // ...
}

void FlightStatsRecorder::logLiftoffStats()
{
    // ...
}

void FlightStatsRecorder::logMainDplStats()
{
    // ...
}

}  // namespace Main
