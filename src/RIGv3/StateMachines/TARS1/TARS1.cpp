/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include "TARS1.h"

#include <RIGv3/Actuators/Actuators.h>
#include <RIGv3/BoardScheduler.h>
#include <RIGv3/Sensors/Sensors.h>
#include <RIGv3/StateMachines/TARS1/TARS1Data.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <scheduler/TaskScheduler.h>

using namespace std::chrono;
using namespace Boardcore;
using namespace RIGv3;
using namespace Common;

TARS1::TARS1()
    : HSM(&TARS1::Ready, miosix::STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::tars1Priority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_TARS);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

bool TARS1::start()
{
    if (!HSM::start())
    {
        LOG_ERR(logger, "Failed to activate TARS1 thread");
        return false;
    }

    return true;
}

void TARS1::updateAndLogAction(Tars1Action action)
{
    lastAction = action;

    auto actionData = Tars1ActionData{
        .timestamp = TimestampTimer::getTimestamp(),
        .action    = action,
    };
    sdLogger.log(actionData);
}

void TARS1::logSample(float pressure, float mass)
{
    auto data = Tars1SampleData{
        .timestamp = TimestampTimer::getTimestamp(),
        .pressure  = pressure,
        .mass      = mass,
    };
    sdLogger.log(data);
}

State TARS1::Ready(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            return HANDLED;
        }

        case EV_ENTRY:
        {
            updateAndLogAction(Tars1Action::READY);
            return HANDLED;
        }

        case MOTOR_START_TARS1:
        {
            return transition(&TARS1::Refueling);
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS1::state_top);
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

State TARS1::Refueling(const Event& event)
{
    Actuators* actuators = getModule<Actuators>();

    switch (event)
    {
        case EV_INIT:
        {
            // Reset TARS state
            cycleCount = 0;

            ventingTime = milliseconds{
                actuators->getValveOpeningTime(ServosList::PRZ_RELEASE_VALVE)};

            fillingTime = milliseconds{
                actuators->getValveOpeningTime(ServosList::PRZ_FILLING_VALVE)};

            stabilizationTime = milliseconds{
                actuators->getValveOpeningTime(ServosList::OX_RELEASE_VALVE)};

            LOG_INFO(logger, "Calculated filling time: {} ms",
                     fillingTime.count());
            LOG_INFO(logger, "Calculated venting time: {} ms",
                     ventingTime.count());

            // Initialize the valves to a known closed state
            actuators->closeValve(ServosList::PRZ_FILLING_VALVE);
            actuators->closeValve(ServosList::PRZ_RELEASE_VALVE);

            LOG_INFO(logger, "TARS1 refueling start");
            updateAndLogAction(Tars1Action::START);

            return transition(&TARS1::RefuelingFilling);
        }

        case EV_ENTRY:
        {
            return HANDLED;
        }

        case MOTOR_MANUAL_ACTION:
        {
            LOG_INFO(logger, "TARS1 stopped because of manual valve action");
            updateAndLogAction(Tars1Action::MANUAL_ACTION_STOP);

            // Close the filling valve as safety measure on manual action
            getModule<Actuators>()->closeValve(ServosList::PRZ_FILLING_VALVE);
            return transition(&TARS1::Ready);
        }

        case MOTOR_STOP_TARS:
        {
            LOG_INFO(logger, "TARS1 stopped because of stop command");
            updateAndLogAction(Tars1Action::MANUAL_STOP);

            getModule<Actuators>()->closeValve(ServosList::PRZ_FILLING_VALVE);
            getModule<Actuators>()->closeValve(ServosList::PRZ_RELEASE_VALVE);
            return transition(&TARS1::Ready);
        }

        case TARS_FILLING_DONE:
        {
            LOG_INFO(logger, "TARS1 stopped because mass target reached");
            updateAndLogAction(Tars1Action::AUTOMATIC_STOP);

            actuators->closeValve(ServosList::PRZ_FILLING_VALVE);
            actuators->closeValve(ServosList::PRZ_RELEASE_VALVE);
            return transition(&TARS1::Ready);
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(delayedEventId);

            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS1::state_top);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State TARS1::RefuelingFilling(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Filling for {} ms", fillingTime.count());
            updateAndLogAction(Tars1Action::FILLING);

            cycleCount++;

            // Open the filling valve
            getModule<Actuators>()->openValveWithTime(
                ServosList::PRZ_FILLING_VALVE, fillingTime.count());

            return HANDLED;
        }

        case MOTOR_PRZ_FIL_CLOSE:
        {
            delayedEventId = EventBroker::getInstance().postDelayed(
                TARS_PRESSURE_STABILIZED, TOPIC_TARS,
                stabilizationTime.count());

            return HANDLED;
        }

        case TARS_PRESSURE_STABILIZED:
        {
            LOG_INFO(logger, "Pressure stabilized, starting venting");

            return transition(&TARS1::RefuelingVenting);
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS1::Refueling);
        }

        case EV_INIT:
        {
            return HANDLED;
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

State TARS1::RefuelingVenting(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Venting for {} ms", ventingTime.count());
            updateAndLogAction(Tars1Action::VENTING);

            getModule<Actuators>()->openValveWithTime(
                ServosList::PRZ_RELEASE_VALVE, ventingTime.count());

            return HANDLED;
        }

        case MOTOR_PRZ_REL_CLOSE:
        {
            delayedEventId = EventBroker::getInstance().postDelayed(
                TARS_PRESSURE_STABILIZED, TOPIC_TARS,
                stabilizationTime.count());

            return HANDLED;
        }

        case TARS_PRESSURE_STABILIZED:
        {
            LOG_INFO(logger, "Venting done");

            if (cycleCount >= Config::TARS1::MAX_FILLING_CYCLES)
            {
                LOG_ERR(logger, "Completed all filling cycles");
                updateAndLogAction(Tars1Action::AUTOMATIC_STOP);
                return transition(&TARS1::Ready);
            }

            return transition(&TARS1::RefuelingFilling);
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS1::Refueling);
        }

        case EV_INIT:
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
