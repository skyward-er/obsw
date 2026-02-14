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
                    LOG_ERR(logger,
                            "Biliquid sequence not set when trying to start "
                            "generic sequence");
                    return UNHANDLED;
                case SEQUENCE_1:
                    return transition(&Biliquid::state_seq_1);
                case SEQUENCE_2:
                    return transition(&Biliquid::state_seq_2_FUEL);
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
            return transition(&Biliquid::state_seq_2_FUEL);
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
            // change the state of the OX and FUEL ereg
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_FUEL);

            // wait before opening the valves to ensure that the ereg algorithm
            // has had enough time to adjust
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_STEP, TOPIC_BILIQUID, Config::Biliquid::DT.count());
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            // fully open OX and fuel valves for SEQ_3_SHUTDOWN_DELAY ms
            getModule<Actuators>()->openServoWithTime(
                ServosList::MAIN_OX_VALVE,
                Config::Biliquid::SEQ_2_SHUTDOWN_DELAY.count());

            getModule<Actuators>()->openServoWithTime(
                ServosList::MAIN_FUEL_VALVE,
                Config::Biliquid::SEQ_2_SHUTDOWN_DELAY.count());

            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_END_SEQUENCE_2, TOPIC_BILIQUID,
                Config::Biliquid::SEQ_2_SHUTDOWN_DELAY.count());
            return HANDLED;
        }

        case BILIQUID_END_SEQUENCE_2:
        {
            return transition(&Biliquid::state_idle);
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
            // Change back the state of the OX and FUEL ereg
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_FUEL);

            getModule<Actuators>()->closeServo(ServosList::MAIN_OX_VALVE);
            getModule<Actuators>()->closeServo(ServosList::MAIN_FUEL_VALVE);
            return HANDLED;
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
            return HANDLED;
        }

        case EV_INIT:
        {
            // change the state of the OX and FUEL ereg
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_FUEL);

            // wait before opening the valves to ensure that the ereg algorithm
            // has had enough time to adjust
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_STEP, TOPIC_BILIQUID, Config::Biliquid::DT.count());
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            getModule<Actuators>()->moveServo(ServosList::MAIN_FUEL_VALVE,
                                              0.4f);

            getModule<Actuators>()->moveServo(ServosList::MAIN_OX_VALVE, 0.4f);

            /*             // change the state of the FUEL ereg
                        EventBroker::getInstance().post(EREG_DISCHARGE,
               TOPIC_EREG_FUEL);

                        // open FUEL valve
                        getModule<Actuators>()->moveServo(
                            ServosList::MAIN_FUEL_VALVE,
                            Config::Biliquid::SEQ_2_FUEL_POSITION); */

            // wait for before opening OX valve
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_SEQ_2_OX, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::SEQ_2_OX_DELAY}.count());

            return HANDLED;
        }

        case BILIQUID_SEQ_2_OX:
        {
            getModule<Actuators>()->animateServo(ServosList::MAIN_FUEL_VALVE,
                                                 1.0f, 500);
            getModule<Actuators>()->animateServo(ServosList::MAIN_OX_VALVE,
                                                 1.0f, 500);

            /* getModule<Actuators>()->closeServo(ServosList::PRZ_OX_VALVE);

            getModule<Actuators>()->closeServo(ServosList::PRZ_FUEL_VALVE);

            getModule<Actuators>()->animateServo(ServosList::PRZ_OX_VALVE, 0.0f,
                                                 10000);

            getModule<Actuators>()->animateServo(ServosList::PRZ_FUEL_VALVE,
                                                 0.0f, 10000); */
            /* return transition(&Biliquid::state_seq_2_OX); */
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_ABORT, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::SEQ_3_SHUTDOWN_DELAY}.count());
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
            // Change back the state of the OX and FUEL ereg
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_FUEL);

            getModule<Actuators>()->closeServo(ServosList::MAIN_FUEL_VALVE);
            getModule<Actuators>()->closeServo(ServosList::MAIN_OX_VALVE);
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
        case EV_INIT:
        {
            updateAndLogStatus(BiliquidState::SEQUENCE_2_OX);
            return HANDLED;
        }

        case EV_ENTRY:
        {
            // change the state of the OX ereg
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_OX);

            // open OX valve
            getModule<Actuators>()->moveServo(
                ServosList::MAIN_OX_VALVE, Config::Biliquid::SEQ_2_OX_POSITION);

            // wait for SEQ_2_SHUTDOWN_DELAY ms before closing all valves
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_END_SEQUENCE_2, TOPIC_BILIQUID,
                milliseconds{Config::Biliquid::SEQ_2_SHUTDOWN_DELAY}.count());

            return HANDLED;
        }

        case BILIQUID_END_SEQUENCE_2:
        {
            return transition(&Biliquid::state_idle);
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
            getModule<Actuators>()->closeServo(ServosList::MAIN_OX_VALVE);
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
            // change the state of the OX and FUEL ereg
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_DISCHARGE, TOPIC_EREG_FUEL);

            // wait before opening the valves to ensure that the ereg algorithm
            // has had enough time to adjust
            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_STEP, TOPIC_BILIQUID, Config::Biliquid::DT.count());
            return HANDLED;
        }

        case BILIQUID_STEP:
        {
            // fully open OX and fuel valves for SEQ_3_SHUTDOWN_DELAY ms
            getModule<Actuators>()->openServoWithTime(
                ServosList::MAIN_OX_VALVE,
                Config::Biliquid::SEQ_3_SHUTDOWN_DELAY.count());

            getModule<Actuators>()->openServoWithTime(
                ServosList::MAIN_FUEL_VALVE,
                Config::Biliquid::SEQ_3_SHUTDOWN_DELAY.count());

            nextEventId = EventBroker::getInstance().postDelayed(
                BILIQUID_END_SEQUENCE_2, TOPIC_BILIQUID,
                Config::Biliquid::SEQ_3_SHUTDOWN_DELAY.count());
            return HANDLED;
        }

        case BILIQUID_END_SEQUENCE_2:
        {
            return transition(&Biliquid::state_idle);
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
            // Change back the state of the OX and FUEL ereg
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(EREG_CLOSE, TOPIC_EREG_FUEL);

            getModule<Actuators>()->closeServo(ServosList::MAIN_OX_VALVE);
            getModule<Actuators>()->closeServo(ServosList::MAIN_FUEL_VALVE);
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
