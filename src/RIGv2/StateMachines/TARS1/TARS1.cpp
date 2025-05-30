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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/TARS1/TARS1Data.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <scheduler/TaskScheduler.h>

using namespace std::chrono;
using namespace Boardcore;
using namespace RIGv2;
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
    TaskScheduler& scheduler = getModule<BoardScheduler>()->tars1();

    uint8_t result =
        scheduler.addTask([this]() { sample(); }, Config::TARS1::SAMPLE_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add TARS1 sample task");
        return false;
    }

    if (!HSM::start())
    {
        LOG_ERR(logger, "Failed to activate TARS1 thread");
        return false;
    }

    return true;
}

void TARS1::sample()
{
    Sensors* sensors = getModule<Sensors>();

    pressureFilter.add(sensors->getOxTankBottomPressure().pressure);
    massFilter.add(sensors->getOxTankWeight().load);
    medianSamples++;

    if (medianSamples == Config::TARS1::MEDIAN_SAMPLE_NUMBER)
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
            massStableCounter = 0;
            previousMass      = 0.0f;
            currentMass       = 0.0f;
            previousPressure  = 0.0f;
            currentPressure   = 0.0f;

            ventingTime = milliseconds{
                actuators->getServoOpeningTime(ServosList::OX_VENTING_VALVE)};

            // Initialize the valves to a known closed state
            actuators->closeServo(ServosList::OX_FILLING_VALVE);
            actuators->closeServo(ServosList::OX_VENTING_VALVE);

            LOG_INFO(logger, "TARS1 refueling start");
            updateAndLogAction(Tars1Action::START);

            return transition(&TARS1::RefuelingWashing);
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
            getModule<Actuators>()->closeServo(ServosList::OX_FILLING_VALVE);
            return transition(&TARS1::Ready);
        }

        case MOTOR_STOP_TARS:
        {
            LOG_INFO(logger, "TARS1 stopped because of stop command");
            updateAndLogAction(Tars1Action::MANUAL_STOP);

            getModule<Actuators>()->closeServo(ServosList::OX_FILLING_VALVE);
            getModule<Actuators>()->closeServo(ServosList::OX_VENTING_VALVE);
            return transition(&TARS1::Ready);
        }

        case TARS_FILLING_DONE:
        {
            LOG_INFO(logger, "TARS1 stopped because mass target reached");
            updateAndLogAction(Tars1Action::AUTOMATIC_STOP);

            actuators->closeServo(ServosList::OX_FILLING_VALVE);
            actuators->closeServo(ServosList::OX_VENTING_VALVE);
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

State TARS1::RefuelingWashing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            LOG_INFO(logger, "Washing the OX tank for {} ms",
                     milliseconds{Config::TARS1::WASHING_OPENING_TIME}.count());
            updateAndLogAction(Tars1Action::WASHING);

            // Start washing
            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_VENTING_VALVE,
                milliseconds{Config::TARS1::WASHING_OPENING_TIME}.count());

            // Wait a bit before opening the filling valve so that the servo
            // don't actuate at the same time
            delayedEventId = EventBroker::getInstance().postDelayed(
                TARS_WASHING_CONTINUE, TOPIC_TARS,
                milliseconds{Config::TARS1::WASHING_TIME_DELAY}.count());

            return HANDLED;
        }

        case TARS_WASHING_CONTINUE:
        {
            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_FILLING_VALVE,
                milliseconds{Config::TARS1::WASHING_OPENING_TIME}.count());
            // Wait double the time we opened the valve for washing completion
            delayedEventId = EventBroker::getInstance().postDelayed(
                TARS_WASHING_DONE, TOPIC_TARS,
                milliseconds{Config::TARS1::WASHING_OPENING_TIME * 2}.count());

            return HANDLED;
        }

        case TARS_WASHING_DONE:
        {
            LOG_INFO(logger, "Washing procedure done");
            return transition(&TARS1::RefuelingFilling);
        }

        case EV_INIT:
        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&TARS1::Refueling);
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
            LOG_INFO(logger, "Filling for {} ms",
                     milliseconds{Config::TARS1::FILLING_OPENING_TIME}.count());
            updateAndLogAction(Tars1Action::FILLING);

            // Open the filling valve
            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_FILLING_VALVE,
                milliseconds{Config::TARS1::FILLING_OPENING_TIME}.count());

            // Wait for pressure stabilization while the filling valve is open
            delayedEventId = EventBroker::getInstance().postDelayed(
                TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                milliseconds{Config::TARS1::FILLING_STABILIZE_WAIT_TIME}
                    .count());
            LOG_INFO(logger, "Waiting for pressure stabilization");

            return HANDLED;
        }

        case TARS_CHECK_PRESSURE_STABILIZE:
        {
            {
                Lock<FastMutex> lock(sampleMutex);
                previousPressure = currentPressure;
                currentPressure  = pressureSample;
            }

            float pressureChange = std::abs(currentPressure - previousPressure);
            bool pressureStable =
                pressureChange < Config::TARS1::PRESSURE_TOLERANCE;

            LOG_INFO(logger, "Pressure change: {} Bar, pressureStable: {}",
                     pressureChange, pressureStable);

            if (pressureStable)
            {
                if (Config::TARS1::STOP_ON_MASS_STABILIZATION)
                {
                    {
                        Lock<FastMutex> lock(sampleMutex);
                        previousMass = currentMass;
                        currentMass  = massSample;
                    }

                    float massChange = std::abs(currentMass - previousMass);
                    bool massStable =
                        massChange < Config::TARS1::MASS_TOLERANCE;

                    LOG_INFO(logger, "Mass change: {} kg, massStable: {}",
                             massChange, massStable);

                    if (massStable)
                    {
                        if (massStableCounter >=
                            Config::TARS1::NUM_MASS_STABLE_ITERATIONS)
                        {
                            EventBroker::getInstance().post(TARS_FILLING_DONE,
                                                            TOPIC_TARS);
                            return HANDLED;
                        }
                        else
                        {
                            massStableCounter++;
                        }
                    }
                    else
                    {
                        massStableCounter = 0;
                    }
                }

                // Proceed to vent the gaseous part of the OX so that we can
                // fill more liquid afterwards
                return transition(&TARS1::RefuelingVenting);
            }
            else
            {
                // Pressure is not stable yet, we can fill more OX
                delayedEventId = EventBroker::getInstance().postDelayed(
                    TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                    milliseconds{Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME}
                        .count());
            }

            return HANDLED;
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
            // Close the filling valve since we're opening it for a long time
            // and we don't want to vent while it's open
            getModule<Actuators>()->closeServo(ServosList::OX_FILLING_VALVE);
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

            getModule<Actuators>()->openServoWithTime(
                ServosList::OX_VENTING_VALVE, ventingTime.count());

            return HANDLED;
        }

        case MOTOR_OX_VEN_CLOSE:
        {
            LOG_INFO(logger, "Venting done");

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
