/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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
#include <RIGv2/Configs/TARS1Config.h>
#include <RIGv2/Sensors/Sensors.h>
#include <common/Events.h>
#include <events/EventBroker.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace RIGv2;
using namespace Common;
using namespace miosix;

TARS1::TARS1(TaskScheduler& scheduler)
    : FSM(&TARS1::state_ready), scheduler{scheduler}
{
    EventBroker::getInstance().subscribe(this, TOPIC_TARS);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

bool TARS1::start()
{
    uint8_t result =
        scheduler.addTask([this]() { sample(); }, Config::TARS1::SAMPLE_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add TARS1 sample task");
        return false;
    }

    if (!ActiveObject::start())
    {
        LOG_ERR(logger, "Failed to activate TARS1 thread");
        return false;
    }

    return true;
}

bool TARS1::isRefueling()
{
    return testState(&TARS1::state_refueling);
}

void TARS1::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logAction(TARS_ACTION_READY);
            break;
        }

        case MOTOR_START_TARS:
        {
            transition(&TARS1::state_refueling);
            break;
        }
    }
}

void TARS1::state_refueling(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    Actuators* actuators   = modules.get<Actuators>();

    switch (event)
    {
        case EV_ENTRY:
        {
            // Reset TARS state
            massStableCounter = 0;
            previousMass      = 0.0f;
            currentMass       = 0.0f;
            previousPressure  = 0.0f;
            currentPressure   = 0.0f;
            // TODO(davide.mor): Add the rest

            // First close all valves
            actuators->closeAllServos();

            LOG_INFO(logger, "TARS start washing");
            logAction(TARS_ACTION_WASHING);

            // Start washing
            actuators->openServoWithTime(ServosList::VENTING_VALVE,
                                         Config::TARS1::WASHING_OPENING_TIME);

            // Wait a bit so that the servo don't actuate at the same time
            Thread::sleep(Config::TARS1::WASHING_TIME_DELAY);

            actuators->openServoWithTime(ServosList::FILLING_VALVE,
                                         Config::TARS1::WASHING_OPENING_TIME);

            // After double the time we opened the valve, move to the next phase
            nextDelayedEventId = EventBroker::getInstance().postDelayed(
                TARS_WASHING_DONE, TOPIC_TARS,
                Config::TARS1::WASHING_OPENING_TIME * 2);

            break;
        }

        case TARS_WASHING_DONE:
        {
            LOG_INFO(logger, "TARS washing done");
            logAction(TARS_ACTION_OPEN_FILLING);

            // Open the filling for a long time
            actuators->openServoWithTime(ServosList::FILLING_VALVE,
                                         Config::TARS1::FILLING_OPENING_TIME);

            nextDelayedEventId = EventBroker::getInstance().postDelayed(
                TARS_PRESSURE_STABILIZED, TOPIC_TARS,
                Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME);

            break;
        }

        case TARS_PRESSURE_STABILIZED:
        {
            LOG_INFO(logger, "TARS check mass");
            logAction(TARS_ACTION_CHECK_MASS);

            // Lock in a new mass value
            {
                Lock<FastMutex> lock(sampleMutex);
                previousMass = currentMass;
                currentMass  = massSample;
            }

            if (std::abs(currentMass - previousMass) <
                Config::TARS1::MASS_TOLERANCE)
            {
                if (massStableCounter >=
                    Config::TARS1::NUM_MASS_STABLE_ITERATIONS)
                {
                    EventBroker::getInstance().post(TARS_FILLING_DONE,
                                                    TOPIC_TARS);
                    break;
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

            LOG_INFO(logger, "TARS open venting");
            logAction(TARS_ACTION_OPEN_VENTING);

            // Open the venting and check for pressure stabilization
            actuators->openServo(ServosList::VENTING_VALVE);

            // Calculate next check time based on the time the valve stays open
            unsigned int nextCheckTime =
                actuators->getServoOpeningTime(ServosList::VENTING_VALVE) +
                Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME * 5;

            nextDelayedEventId = EventBroker::getInstance().postDelayed(
                TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS, nextCheckTime);
            break;
        }

        case TARS_CHECK_PRESSURE_STABILIZE:
        {
            LOG_INFO(logger, "TARS check pressure");
            logAction(TARS_ACTION_CHECK_PRESSURE);

            {
                Lock<FastMutex> lock(sampleMutex);
                previousPressure = currentPressure;
                currentPressure  = pressureSample;
            }

            if (std::abs(currentPressure - previousPressure) <
                Config::TARS1::PRESSURE_TOLERANCE)
            {
                // The pressure is stable, do another cicle
                nextDelayedEventId = EventBroker::getInstance().postDelayed(
                    TARS_PRESSURE_STABILIZED, TOPIC_TARS,
                    Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME);
            }
            else
            {
                // Schedule a new check
                nextDelayedEventId = EventBroker::getInstance().postDelayed(
                    TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                    Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME);
            }

            break;
        }

        case TARS_FILLING_DONE:
        {
            LOG_INFO(logger, "TARS filling done");
            logAction(TARS_ACTION_AUTOMATIC_STOP);

            actuators->closeAllServos();
            transition(&TARS1::state_ready);
            break;
        }

        case MOTOR_MANUAL_ACTION:
        {
            LOG_INFO(logger, "TARS manual stop");
            logAction(TARS_ACTION_MANUAL_STOP);

            // Disable next event
            EventBroker::getInstance().removeDelayed(nextDelayedEventId);
            transition(&TARS1::state_ready);
            break;
        }

        case MOTOR_START_TARS:
        {
            LOG_INFO(logger, "TARS manual stop");
            logAction(TARS_ACTION_MANUAL_STOP);

            // The user requested that we stop
            modules.get<Actuators>()->closeAllServos();
            // Disable next event
            EventBroker::getInstance().removeDelayed(nextDelayedEventId);
            transition(&TARS1::state_ready);
        }
    }
}

void TARS1::sample()
{
    ModuleManager& modules = ModuleManager::getInstance();
    Sensors* sensors       = modules.get<Sensors>();

    float pressure = sensors->getTankBottomPress().pressure;
    float mass     = sensors->getTankWeight().load;

    // TODO(davide.mor): Perform filtering on input data

    logSample(pressure, mass);

    {
        Lock<FastMutex> lock(sampleMutex);
        pressureSample = pressure;
        massSample     = mass;
    }
}

void TARS1::logAction(TarsActionType action)
{
    TarsActionData data = {TimestampTimer::getTimestamp(), action};
    sdLogger.log(data);
}

void TARS1::logSample(float pressure, float mass)
{
    TarsSampleData data = {TimestampTimer::getTimestamp(), pressure, mass};
    sdLogger.log(data);
}