/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include "FlightModeManager.h"

#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace Boardcore;

namespace Parafoil
{

void FlightModeManager::state_on_ground(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            return logStatus(FlightModeManagerState::ON_GROUND);
        }
        case EV_LIFTOFF:
        {
            return transition(&FlightModeManager::state_flying);
        }
        case EV_TC_TEST_MODE:
        {
            return transition(&FlightModeManager::state_debug);
        }
    }
}

void FlightModeManager::state_flying(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            return logStatus(FlightModeManagerState::FLYING_STATE);
        }
    }
}

void FlightModeManager::state_debug(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            return logStatus(FlightModeManagerState::DEBUG_STATE);
        }
        case EV_TC_EXIT_TEST_MODE:
        {
            return transition(&FlightModeManager::state_on_ground);
        }
    }
}

FlightModeManager::FlightModeManager()
    : FSM(&FlightModeManager::state_on_ground)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

FlightModeManager::~FlightModeManager()
{
    EventBroker::getInstance().unsubscribe(this);
}

void FlightModeManager::logStatus(FlightModeManagerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(state);
}

}  // namespace Parafoil
