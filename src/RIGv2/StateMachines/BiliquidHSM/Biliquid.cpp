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

bool Biliquid::start()
{
    /* TaskScheduler& scheduler = getModule<BoardScheduler>()->biliquid();

    uint8_t result =
        scheduler.addTask([this]() { sample(); }, Config::TARS3::SAMPLE_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add TARS3 sample task");
        return false;
    } */

    if (!HSM::start())
    {
        LOG_ERR(logger, "Failed to activate Biliquid HSM thread");
        return false;
    }

    return true;
}

BiliquidState Biliquid::getState() { return state; }
ValveSequenceList Biliquid::getCurrentSequence() { return currentSequence; }

State Biliquid::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::IDLE);
            return HANDLED;
        }

        case EV_ENTRY:
        {
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

        case BILIQUID_READY:
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
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::READY);

            getModule<Actuators>()->closeAllServos();

            return HANDLED;
        }

        case EV_ENTRY:
        {
            return HANDLED;
        }

        case BILIQUID_START_SEQUENCE_GENERIC:
        {
            switch (currentSequence)
            {
                case SEQUENCE_0:
                    return transition(&Biliquid::state_seq_0);
                case SEQUENCE_1:
                    return transition(&Biliquid::state_seq_1);
                case SEQUENCE_2:
                    return transition(&Biliquid::state_seq_2);
                case SEQUENCE_3:
                    return transition(&Biliquid::state_seq_3);
                default:
                    break;
            }
        }

        case BILIQUID_START_SEQUENCE_1:
        {
            return transition(&Biliquid::state_seq_1);
        }

        case BILIQUID_START_SEQUENCE_2:
        {
            return transition(&Biliquid::state_seq_2);
        }

        case BILIQUID_START_SEQUENCE_3:
        {
            return transition(&Biliquid::state_seq_3);
        }

        case BILIQUID_ABORT:
        {
            return transition(&Biliquid::state_idle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_idle);
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

// cft sequence
State Biliquid::state_seq_0(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::SEQUENCE_0);
            return HANDLED;
        }

        case EV_ENTRY:
        {
            getModule<Actuators>()->openOxSolenoidWithTime(6000);
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_STEP, TOPIC_BILIQUID, milliseconds{3000ms}.count());
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            getModule<Actuators>()->openFuelSolenoidWithTime(3000);
            return HANDLED;
        }

        case BILIQUID_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&Biliquid::state_idle);
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->closeOxSolenoid();
            getModule<Actuators>()->closeFuelSolenoid();
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_ready);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

// Purge sequence
State Biliquid::state_seq_1(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::SEQUENCE_1);
            return HANDLED;
        }

        case EV_ENTRY:
        {
            getModule<Actuators>()->openOxSolenoidWithTime(5000);
            getModule<Actuators>()->openFuelSolenoidWithTime(4500);
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            return HANDLED;
        }

        case BILIQUID_ABORT:
        {
            return transition(&Biliquid::state_idle);
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->closeOxSolenoid();
            getModule<Actuators>()->closeFuelSolenoid();
            getModule<Actuators>()->stopSparkPlug();
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_ready);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

// sft sequence 1
State Biliquid::state_seq_2(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::SEQUENCE_2);
            return HANDLED;
        }

        case EV_ENTRY:
        {
            getModule<Actuators>()->openOxSolenoidWithTime(2500);
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_STEP, TOPIC_BILIQUID, milliseconds{100ms}.count());
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            getModule<Actuators>()->openFuelSolenoidWithTime(2000);
            getModule<Actuators>()->startSparkPlugWithTime(500);
            return HANDLED;
        }

        case BILIQUID_ABORT:
        {
            EventBroker::getInstance().removeDelayed(nextEventId);
            return transition(&Biliquid::state_idle);
        }
        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_ready);
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->closeOxSolenoid();
            getModule<Actuators>()->closeFuelSolenoid();
            getModule<Actuators>()->stopSparkPlug();
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
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::SEQUENCE_3);
            return HANDLED;
        }

        case EV_ENTRY:
        {
            // start stepping
            iterationCount = 0;
            EventBroker::getInstance().post(BILIQUID_OX, TOPIC_BILIQUID);
            return HANDLED;
        }

        case BILIQUID_OX:
        {
            if (iterationCount >= Config::Biliquid::maxIterationCount)
            {
                transition(&Biliquid::state_idle);
                return HANDLED;
            }

            getModule<Actuators>()->openOxSolenoidWithTime(1300);
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_FUEL, TOPIC_BILIQUID, milliseconds{100ms}.count());
            return HANDLED;
        }

        case BILIQUID_FUEL:
        {
            getModule<Actuators>()->openFuelSolenoidWithTime(800);
            getModule<Actuators>()->startSparkPlugWithTime(500);

            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_PURGE, TOPIC_BILIQUID, milliseconds{1700ms}.count());
            return HANDLED;
        }

        case BILIQUID_PURGE:
        {
            getModule<Actuators>()->openOxSolenoidWithTime(2000);
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_OX, TOPIC_BILIQUID, milliseconds{2500ms}.count());

            iterationCount++;
            return HANDLED;
        }

        case BILIQUID_ABORT:
        {
            return transition(&Biliquid::state_idle);
        }

        case EV_EMPTY:
        {
            return tranSuper(&Biliquid::state_ready);
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->closeOxSolenoid();
            getModule<Actuators>()->closeFuelSolenoid();
            getModule<Actuators>()->stopSparkPlug();
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

bool Biliquid::setSequence(ValveSequenceList sequence)
{
    currentSequence = sequence;
    return true;
};

void Biliquid::updateAndLogStatus(BiliquidState state)
{
    this->state = state;

    // printf("Biliquid state updated to %s\n", to_string(state).c_str());

    BiliquidData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}

}  // namespace RIGv2
