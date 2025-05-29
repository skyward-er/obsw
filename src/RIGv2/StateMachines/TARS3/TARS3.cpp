/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "TARS3.h"

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Configs/TARS3Config.h>
#include <RIGv2/Sensors/Sensors.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/Registry/RegistryFrontend.h>

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;

namespace RIGv2
{

TARS3::TARS3()
    : HSM(&TARS3::Ready, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::TARS_PRIORITY)
{
    EventBroker::getInstance().subscribe(this, TOPIC_TARS);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

bool TARS3::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getTars1Scheduler();

    uint8_t result =
        scheduler.addTask([this]() { sample(); }, Config::TARS3::SAMPLE_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add TARS3 sample task");
        return false;
    }

    if (!HSM::start())
    {
        LOG_ERR(logger, "Failed to activate TARS3 thread");
        return false;
    }

    return true;
}

void TARS3::sample()
{
    Sensors* sensors = getModule<Sensors>();

    pressureFilter.add(sensors->getOxTankBottomPressure().pressure);
    massFilter.add(sensors->getOxTankWeight().load);
    medianSamples++;

    if (medianSamples == Config::TARS3::MEDIAN_SAMPLE_NUMBER)
    {
        float pressure = pressureFilter.calcMedian();
        float mass     = massFilter.calcMedian();
        medianSamples  = 0;

        logSample(pressure, mass);

        {
            Lock<FastMutex> lock(sampleMutex);
            currentPressure = pressure;
            currentMass     = mass;
        }
    }
}

void TARS3::updateAndLogAction(Tars3Action action)
{
    lastAction = action;

    auto actionData = Tars3ActionData{
        .timestamp = TimestampTimer::getTimestamp(),
        .action    = action,
    };
    sdLogger.log(actionData);
}

void TARS3::logSample(float pressure, float mass)
{
    auto data = Tars3SampleData{
        .timestamp = TimestampTimer::getTimestamp(),
        .pressure  = pressure,
        .mass      = mass,
    };
    sdLogger.log(data);
}

State TARS3::Ready(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            return HANDLED;
        }

        case EV_ENTRY:
        {
            updateAndLogAction(Tars3Action::READY);
            return HANDLED;
        }

        case MOTOR_START_TARS3:
        {
            return transition(&TARS3::Refueling);
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS3::state_top);
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

State TARS3::Refueling(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            auto* actuators = getModule<Actuators>();

            currentPressure = 0.0f;
            currentMass     = 0.0f;
            fillingTime     = milliseconds{
                actuators->getServoOpeningTime(ServosList::OX_FILLING_VALVE)};
            ventingTime = milliseconds{
                actuators->getServoOpeningTime(ServosList::OX_VENTING_VALVE)};

            // Initialize the valves to a known closed state
            actuators->closeServo(ServosList::OX_FILLING_VALVE);
            actuators->closeServo(ServosList::OX_VENTING_VALVE);

            LOG_INFO(logger, "TARS3 cold refueling start");
            updateAndLogAction(Tars3Action::START);

            return transition(&TARS3::RefuelingWait);
        }

        case EV_ENTRY:
        {
            return HANDLED;
        }

        case MOTOR_MANUAL_ACTION:
        {
            LOG_INFO(logger, "TARS3 stopped because of manual valve action");
            updateAndLogAction(Tars3Action::MANUAL_ACTION_STOP);

            // Close the filling valve as safety measure on manual action
            getModule<Actuators>()->closeServo(ServosList::OX_FILLING_VALVE);
            return transition(&TARS3::Ready);
        }

        case MOTOR_STOP_TARS:
        {
            LOG_INFO(logger, "TARS3 stopped because of stop command");
            updateAndLogAction(Tars3Action::MANUAL_STOP);

            getModule<Actuators>()->closeServo(ServosList::OX_FILLING_VALVE);
            getModule<Actuators>()->closeServo(ServosList::OX_VENTING_VALVE);
            return transition(&TARS3::Ready);
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(delayedEventId);

            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS3::state_top);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State TARS3::RefuelingWait(const Event& event)
{
    float pressure = 0.0f;
    float mass     = 0.0f;
    {
        Lock<FastMutex> lock(sampleMutex);
        pressure = currentPressure;
        mass     = currentMass;
    }

    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Waiting for system to stabilize");
            updateAndLogAction(Tars3Action::WAITING);

            EventBroker::getInstance().postDelayed(
                TARS_PRESSURE_STABILIZED, TOPIC_TARS,
                milliseconds{Config::TARS3::WAIT_BETWEEN_CYCLES}.count());

            return HANDLED;
        }

        case TARS_PRESSURE_STABILIZED:
        {
            auto* registry = getModule<Registry>();

            float massTarget = registry->getOrSetDefaultUnsafe(
                CONFIG_ID_TARS3_MASS_TARGET,
                Config::TARS3::DEFAULT_MASS_TARGET);
            float pressureTarget = registry->getOrSetDefaultUnsafe(
                CONFIG_ID_TARS3_PRESSURE_TARGET,
                Config::TARS3::DEFAULT_PRESSURE_TARGET);

            // OX below target temperature
            bool coldEnough = pressure <= pressureTarget;
            // Mass target reached
            bool massTargetReached = mass >= massTarget;

            LOG_INFO(logger,
                     "Pressure: {} bar, Mass: {} kg, Cold enough: {}, Mass "
                     "target reached: {}",
                     pressure, mass, coldEnough, massTargetReached);

            if (coldEnough && !massTargetReached)
            {
                // OX is cold enough to fill, if mass target not reached
                return transition(&TARS3::RefuelingFilling);
            }
            else if (!coldEnough)
            {
                // OX is too hot, vent to lower the temperature
                return transition(&TARS3::RefuelingVenting);
            }
            else
            {
                // OX is cold enough and mass target has been reached
                // Nothing to do for now, keep waiting
                EventBroker::getInstance().postDelayed(
                    TARS_PRESSURE_STABILIZED, TOPIC_TARS,
                    milliseconds{Config::TARS3::WAIT_BETWEEN_CYCLES}.count());

                return HANDLED;
            }
        }

        case EV_INIT:
        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS3::Refueling);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State TARS3::RefuelingFilling(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Filling for {} ms", fillingTime.count());
            updateAndLogAction(Tars3Action::FILLING);

            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_FILLING_VALVE, fillingTime.count());

            return HANDLED;
        }

        case MOTOR_OX_FIL_CLOSE:
        {
            LOG_INFO(logger, "Filling done");

            return transition(&TARS3::RefuelingWait);
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS3::Refueling);
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

State TARS3::RefuelingVenting(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Venting for {} ms", ventingTime.count());
            updateAndLogAction(Tars3Action::VENTING);

            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_VENTING_VALVE, ventingTime.count());

            return HANDLED;
        }

        case MOTOR_OX_VEN_CLOSE:
        {
            LOG_INFO(logger, "Venting done");

            return transition(&TARS3::RefuelingWait);
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS3::Refueling);
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

}  // namespace RIGv2
