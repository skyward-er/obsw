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

#include <RIGv2/Configs/SchedulerConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace RIGv2;
using namespace Common;
using namespace miosix;

TARS1::TARS1()
    : FSM(&TARS1::state_ready, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::TARS1_PRIORITY)
{
    EventBroker::getInstance().subscribe(this, TOPIC_TARS);
    EventBroker::getInstance().subscribe(this, TOPIC_TARS_ASYNC);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

bool TARS1::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getTars1Scheduler();

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

bool TARS1::isRefueling() { return testState(&TARS1::state_refueling); }

void TARS1::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logAction(TarsActionType::READY);
            break;
        }

        case MOTOR_START_TARS:
        {
            transition(&TARS1::state_refueling);
            break;
        }
    }
}

#define ASYNC_BEGIN(topic, event)                          \
    constexpr auto _ASYNC_TOPIC      = topic;              \
    constexpr auto _ASYNC_CONT_EVENT = event;              \
    static int _lc                   = 0;                  \
    if (event == TARS_ASYNC_CONTINUE || event == EV_ENTRY) \
    {                                                      \
        switch (_lc)                                       \
        {                                                  \
            case 0:
#define ASYNC_END() \
    }               \
    return;         \
    }

#define ASYNC_WAIT_FOR(t)                                                \
    _lc = __LINE__;                                                      \
    case __LINE__:                                                       \
        if (event != __LINE__)                                           \
        {                                                                \
            nextDelayedEventId = EventBroker::getInstance().postDelayed( \
                _ASYNC_CONT_EVENT, _ASYNC_TOPIC, t);                     \
            return;                                                      \
        }

void TARS1::state_refueling(const Event& event)
{
    Actuators* actuators = getModule<Actuators>();

    ASYNC_BEGIN(TOPIC_TARS_ASYNC, TARS_ASYNC_CONTINUE)

    // Reset TARS state
    massStableCounter = 0;
    previousMass      = 0.0f;
    currentMass       = 0.0f;
    previousPressure  = 0.0f;
    currentPressure   = 0.0f;

    // Start with all vavles closed
    actuators->closeAllServos();

    LOG_INFO(logger, "TARS start washing");
    logAction(TarsActionType::WASHING);

    // Start washing procedure
    actuators->openServoWithTime(ServosList::VENTING_VALVE,
                                 Config::TARS1::WASHING_OPENING_TIME);
    // Wait a bit so that the servo don't actuate at the same time
    ASYNC_WAIT_FOR(Config::TARS1::WASHING_TIME_DELAY);
    actuators->openServoWithTime(ServosList::FILLING_VALVE,
                                 Config::TARS1::WASHING_OPENING_TIME);

    // Wait after washing to ensure the valves have closed
    ASYNC_WAIT_FOR(Config::TARS1::WASHING_OPENING_TIME * 2);

    // Washing complete
    LOG_INFO(logger, "TARS washing done");
    logAction(TarsActionType::OPEN_FILLING);

    // Open the filling valve for the whole refueling process
    actuators->openServoWithTime(ServosList::FILLING_VALVE,
                                 Config::TARS1::FILLING_OPENING_TIME);
    // Wait for the system to stabilize
    ASYNC_WAIT_FOR(Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME);

    while (true)
    {
        LOG_INFO(logger, "TARS check mass");
        logAction(TarsActionType::CHECK_MASS);

        // Lock in a new mass value
        {
            Lock<FastMutex> lock(sampleMutex);
            previousMass = currentMass;
            currentMass  = massSample;
        }

        if (Config::TARS1::STOP_ON_MASS_STABILIZATION)
        {
            float massDelta = std::abs(currentMass - previousMass);
            // Update the mass stabilization counter
            if (massDelta < Config::TARS1::MASS_TOLERANCE)
                massStableCounter++;
            else
                massStableCounter = 0;

            // If the mass is stable, stop the refueling
            if (massStableCounter >= Config::TARS1::NUM_MASS_STABLE_ITERATIONS)
            {
                massStableCounter = 0;
                EventBroker::getInstance().post(TARS_FILLING_DONE, TOPIC_TARS);
                break;
            }
        }

        LOG_INFO(logger, "TARS open venting");
        logAction(TarsActionType::OPEN_VENTING);

        // Open venting valve
        actuators->openServo(ServosList::VENTING_VALVE);
        // Wait for system stabilization based on valve opening time
        ASYNC_WAIT_FOR(
            actuators->getServoOpeningTime(ServosList::VENTING_VALVE) +
            Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME * 5)  // wait (5000ms)

        // If the pressure is not stable, keep waiting until it is
        while (true)
        {
            LOG_INFO(logger, "TARS check pressure");
            logAction(TarsActionType::CHECK_PRESSURE);

            {
                Lock<FastMutex> lock(sampleMutex);
                previousPressure = currentPressure;
                currentPressure  = pressureSample;
            }

            // Stop waiting once the pressure is stable
            if (std::abs(currentPressure - previousPressure) <
                Config::TARS1::PRESSURE_TOLERANCE)
                break;

            // Pressure is not stable yet, wait more
            ASYNC_WAIT_FOR(Config::TARS1::PRESSURE_STABILIZE_WAIT_TIME);
        }
    }

    ASYNC_END()

    switch (event)
    {
        case TARS_FILLING_DONE:
        {
            LOG_INFO(logger, "TARS filling done");
            logAction(TarsActionType::AUTOMATIC_STOP);

            actuators->closeAllServos();
            transition(&TARS1::state_ready);
            break;
        }

        case MOTOR_MANUAL_ACTION:
        {
            LOG_INFO(logger, "TARS manual stop");
            logAction(TarsActionType::MANUAL_STOP);

            // Disable next event
            EventBroker::getInstance().removeDelayed(nextDelayedEventId);
            transition(&TARS1::state_ready);
            break;
        }

        case MOTOR_START_TARS:
        {
            LOG_INFO(logger, "TARS manual stop");
            logAction(TarsActionType::MANUAL_STOP);

            // The user requested that we stop
            getModule<Actuators>()->closeAllServos();
            // Disable next event
            EventBroker::getInstance().removeDelayed(nextDelayedEventId);
            transition(&TARS1::state_ready);
            break;
        }
    }
}

void TARS1::sample()
{
    Sensors* sensors = getModule<Sensors>();

    pressureFilter.add(sensors->getBottomTankPressLastSample().pressure);
    massFilter.add(sensors->getTankWeightLastSample().load);
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
