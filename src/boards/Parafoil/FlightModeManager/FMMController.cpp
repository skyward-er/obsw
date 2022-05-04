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

#include "FMMController.h"

#include <common/events/Events.h>
#include <events/EventBroker.h>
#include <miosix.h>

using namespace Boardcore;

namespace Parafoil
{
FMMController::FMMController() : FSM(&FMMController::state_on_ground)
{
    memset(&status, 0, sizeof(FMMControllerStatus));
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

FMMController::~FMMController()
{
    EventBroker::getInstance().unsubscribe(this);
}

void FMMController::state_on_ground(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            // ...

            logStatus(ON_GROUND);

            LOG_DEBUG(logger, "[FMM] entering state on_ground\n");
            break;
        }
        case EV_EXIT:
        {
            // ...

            LOG_DEBUG(logger, "[FMM] exiting state on_ground\n");
            break;
        }
        case EV_LIFTOFF:
        {
            transition(&FMMController::state_flying);
            break;
        }
        case EV_TC_TEST_MODE:
        {
            transition(&FMMController::state_debug);
            break;
        }
        default:
        {
            break;
        }
    }
}

void FMMController::state_flying(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            // ...

            logStatus(FLYING_STATE);

            LOG_DEBUG(logger, "[FMM] entering state flying\n");
            break;
        }
        case EV_EXIT:
        {
            // ...

            LOG_DEBUG(logger, "[FMM] exiting state flying\n");
            break;
        }

        default:
        {
            break;
        }
    }
}

void FMMController::state_debug(const Event& ev)
{
    switch (ev)
    {
        case EV_ENTRY:
        {
            // ...

            logStatus(DEBUG_STATE);

            LOG_DEBUG(logger, "[FMM] entering state debug\n");
            break;
        }
        case EV_EXIT:
        {
            // ...

            LOG_DEBUG(logger, "[FMM] exiting state debug\n");
            break;
        }
        case EV_TC_EXIT_TEST_MODE:
        {
            transition(&FMMController::state_on_ground);
            break;
        }
        default:
        {
            break;
        }
    }
}

/* --- LOGGER --- */

void FMMController::logStatus(FMMControllerState state)
{
    status.state = state;
    logStatus();
}

void FMMController::logStatus()
{
    status.timestamp = miosix::getTick();
    SDlogger->log(status);
}

}  // namespace Parafoil
