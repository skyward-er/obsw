/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include "Biliquid.h"

#include <RIGv2/Configs/BiliquidConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace std::chrono;

namespace RIGv2
{

Biliquid::Biliquid()
    : HSM(&Biliquid::state_idle, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::biliquidPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_BILIQUID);
}

BiliquidState Biliquid::getState() { return state; }

State Biliquid::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(BiliquidState::IDLE);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        case EV_INIT:
        {
            return transition(&Biliquid::state_ready);
        }

        case TMTC_RESET_BOARD:
        {
            reboot();
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State Biliquid::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(BiliquidState::READY);

            // close all valves

            return HANDLED;
        }

        case BILIQUID_START_SEQUENCE_1:
        {
            return transition(&Biliquid::state_seq_1);
        }

        case BILIQUID_START_SEQUENCE_2:
        {
            return transition(&Biliquid::state_seq_2_FUEL);
        }

        case BILIQUID_START_SEQUENCE_3:
        {
            return transition(&Biliquid::state_seq_3);
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State Biliquid::state_seq_1(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(BiliquidState::SEQUENCE_1);

            // start stepping
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_STEP, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::DT}.count());
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            if (stepCount < Config::Biliquid::maxStepCount)
            {  // move the valve to the next step

                // update the number of steps taken before stepping again
                stepCount++;

                nextEventId = EventBroker::getInstance().postDelayed(
                    BILIQUID_STEP, TOPIC_BILIQUID,
                    milliseconds{Config::Biliquid::DT}.count());

                return HANDLED;
            }
            else
                return transition(&Biliquid::state_idle);
        }

        case BILIQUID_END_SEQUENCE_1:
        {
            // reset step count for new sequence
            stepCount = 0;
            return tranSuper(&Biliquid::state_idle);
        }

        case BILIQUID_ABORT:
        {
            // reset step count and remove delayed event
            stepCount = 0;
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&Biliquid::state_aborted);
        }

        case EV_EXIT:
        {
            // close fuel and ox valves
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State Biliquid::state_seq_2_FUEL(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // open FUEL valve

            // wait for SEQ_2_DELAY ms before opening OX valve
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_SEQ_2_OX, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::SEQ_2_OX_DELAY}.count());
            return HANDLED;
        }

        case BILIQUID_SEQ_2_OX:
        {
            return transition(&Biliquid::state_seq_2_OX);
        }

        case BILIQUID_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&Biliquid::state_aborted);
        }
        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        case EV_EXIT:
        {
            // close fuel valve

            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State Biliquid::state_seq_2_OX(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // open OX valve

            // wait for SEQ_2_SHUTDOWN_DELAY ms before closing all valves
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_END_SEQUENCE_2, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::SEQ_2_SHUTDOWN_DELAY}.count());
        }

        case BILIQUID_END_SEQUENCE_2:
        {
            return transition(&Biliquid::state_idle);
        }

        case BILIQUID_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&Biliquid::state_aborted);
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        case EV_EXIT:
        {
            // close OX valve
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State Biliquid::state_seq_3(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // animate open OX and fuel valves

            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_SEQ_3_SHUTDOWN, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::SEQ_3_SHUTDOWN_DELAY}.count());
        }

        case BILIQUID_SEQ_3_SHUTDOWN:
        {
            return transition(&Biliquid::state_idle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        case EV_EXIT:
        {
            // close fuel and OX valves
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State Biliquid::state_aborted(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(BiliquidState::ABORTED);
            return HANDLED;
        }

        case BILIQUID_RESET:
        {
            return transition(&Biliquid::state_idle);
        }
        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_top);
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

void Biliquid::updateAndLogStatus(BiliquidState state)
{
    this->state = state;

    BiliquidData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}

}  // namespace RIGv2
