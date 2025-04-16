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
            pressureSample = pressure;
            massSample     = mass;
        }
    }
}

void TARS3::logAction(Tars3Action action, float data)
{
    auto actionData = Tars3ActionData{
        .timestamp = TimestampTimer::getTimestamp(),
        .action    = action,
        .data      = data,
    };
    sdLogger.log(actionData);
}

void TARS3::logSample(float pressure, float mass)
{
    Tars3SampleData data = {TimestampTimer::getTimestamp(), pressure, mass};
    sdLogger.log(data);
}

State TARS3::Ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logAction(Tars3Action::READY);
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

        default:
        {
            return UNHANDLED;
        }
    }
}

State TARS3::Refueling(const Event& event)
{
    Actuators* actuators = getModule<Actuators>();

    switch (event)
    {
        case EV_INIT:
        {
            previousPressure = 0.0f;
            currentPressure  = 0.0f;
            fillingTime      = Config::TARS3::FILLING_TIME;
            ventingTime      = Config::TARS3::VENTING_TIME;

            // Initialize the valves to a known closed state
            actuators->closeAllServos();

            LOG_INFO(logger, "TARS3 cold refueling start");
            logAction(Tars3Action::START);

            return transition(&TARS3::RefuelingWaitAfterCycle);
        }

        case EV_ENTRY:
        {
            return HANDLED;
        }

        case MOTOR_MANUAL_ACTION:
        {
            LOG_INFO(logger, "TARS3 stopped because of manual valve action");
            logAction(Tars3Action::MANUAL_ACTION_STOP);

            return transition(&TARS3::Ready);
        }

        case MOTOR_STOP_TARS:
        {
            LOG_INFO(logger, "TARS3 stopped because of manual stop");
            logAction(Tars3Action::MANUAL_STOP);

            actuators->closeAllServos();
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

State TARS3::RefuelingWaitAfterCycle(const Event& event)
{
    {
        Lock<FastMutex> lock(sampleMutex);
        previousPressure = currentPressure;
        currentPressure  = pressureSample;
    }

    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Waiting for system to stabilize after cycle");
            logAction(Tars3Action::WAITING_CYCLE);

            EventBroker::getInstance().postDelayed(
                TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                milliseconds{Config::TARS3::WAIT_BETWEEN_CYCLES}.count());

            return HANDLED;
        }

        case TARS_CHECK_PRESSURE_STABILIZE:
        {
            float pressureChange = std::abs(currentPressure - previousPressure);

            if (pressureChange < Config::TARS3::PRESSURE_CHANGE_TOLERANCE)
            {
                // The pressure is stable
                LOG_INFO(logger,
                         "Pressure is stable, proceeding with the cycle");
                logAction(Tars3Action::PRESSURE_STABLE_CYCLE, pressureChange);

                EventBroker::getInstance().post(TARS_PRESSURE_STABILIZED,
                                                TOPIC_TARS);
            }
            else
            {
                // Pressure is not stable yet, schedule a new check
                LOG_INFO(logger,
                         "Waiting for pressure to stabilize before cycle");
                logAction(Tars3Action::WAITING_CYCLE, pressureChange);

                delayedEventId = EventBroker::getInstance().postDelayed(
                    TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                    milliseconds{Config::TARS3::PRESSURE_STABILIZE_WAIT_TIME}
                        .count());
            }

            return HANDLED;
        }

        case TARS_PRESSURE_STABILIZED:
        {
            if (currentPressure < Config::TARS3::PRESSURE_LOWER_RANGE)
            {
                // If OX is cold enough, start a new filling-venting cycle
                return transition(&TARS3::RefuelingFilling);
            }
            else
            {
                // OX is too hot, vent to lower the temperature
                return transition(&TARS3::RefuelingVenting);
            }
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
            LOG_INFO(logger, "Filling");
            logAction(Tars3Action::FILLING, fillingTime.count());

            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_FILLING_VALVE, fillingTime.count());

            return HANDLED;
        }

        case MOTOR_OX_FIL_CLOSE:
        {
            LOG_INFO(logger, "Filling done");

            return transition(&TARS3::RefuelingWaitAfterFilling);
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

State TARS3::RefuelingWaitAfterFilling(const Event& event)
{
    {
        Lock<FastMutex> lock(sampleMutex);
        previousPressure = currentPressure;
        currentPressure  = pressureSample;
    }

    switch (event)
    {
        case EV_ENTRY:
        case TARS_CHECK_PRESSURE_STABILIZE:
        {
            float pressureChange = std::abs(currentPressure - previousPressure);

            if (pressureChange < Config::TARS3::PRESSURE_CHANGE_TOLERANCE)
            {
                // The pressure is stable
                LOG_INFO(logger,
                         "Pressure is stable after filling, proceeding");
                logAction(Tars3Action::PRESSURE_STABLE_FILLING, pressureChange);

                EventBroker::getInstance().post(TARS_PRESSURE_STABILIZED,
                                                TOPIC_TARS);
            }
            else
            {
                // Pressure is not stable yet, schedule a new check
                LOG_INFO(logger,
                         "Waiting for pressure to stabilize after filling");
                logAction(Tars3Action::WAITING_FILLING, pressureChange);

                delayedEventId = EventBroker::getInstance().postDelayed(
                    TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                    milliseconds{Config::TARS3::PRESSURE_STABILIZE_WAIT_TIME}
                        .count());
            }

            return HANDLED;
        }

        case TARS_PRESSURE_STABILIZED:
        {
            if (currentPressure > Config::TARS3::PRESSURE_UPPER_RANGE)
            {
                // OX is hot, vent to lower the temperature and end the cycle
                return transition(&TARS3::RefuelingVenting);
            }
            else
            {
                // OX is still cold enough, keep filling
                return transition(&TARS3::RefuelingFilling);
            }
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

State TARS3::RefuelingVenting(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Venting");
            logAction(Tars3Action::VENTING, ventingTime.count());

            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_VENTING_VALVE, ventingTime.count());

            return HANDLED;
        }

        case MOTOR_OX_VEN_CLOSE:
        {
            LOG_INFO(logger, "Venting done");

            return transition(&TARS3::RefuelingWaitAfterCycle);
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

}  // namespace RIGv2
