/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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
#include <RIG/Actuators/Actuators.h>
#include <RIG/Sensors/Sensors.h>
#include <RIG/StateMachines/TARS1/TARS1.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace miosix;
using namespace Common;

namespace RIG
{
TARS1::TARS1(TaskScheduler* sched) : FSM(&TARS1::state_idle), scheduler(sched)
{
    EventBroker::getInstance().subscribe(this, TOPIC_TARS);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

bool TARS1::start()
{
    // Add task to the task scheduler to sample and compute the average of the
    // sensors
    uint8_t result = scheduler->addTask([=]() { sample(); },
                                        Config::TARS::TARS_UPDATE_PERIOD,
                                        TaskScheduler::Policy::RECOVER);

    return ActiveObject::start() && result != 0;
}

void TARS1::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(TARS1State::IDLE);
        }
        case FMM_INIT_OK:
        {
            return transition(&TARS1::state_ready);
        }
    }
}

void TARS1::state_ready(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            // Reset all the stats and samples
            {
                Lock<FastMutex> lock(mutex);
                samples.mass     = 0;
                samples.pressure = 0;

                massAtVenting         = 0;
                previousMassAtVenting = 0;
                previousPressure      = 0;

                massStableCounter = 0;
                sampleCounter     = 0;

                filteredData.mass.reset();
                filteredData.pressure.reset();
            }
            return logStatus(TARS1State::READY);
        }
        case MOTOR_START_TARS:
        {
            // By default close all valves
            modules.get<Actuators>()->closeAllServo();
            return transition(&TARS1::state_washing);
        }
    }
}

void TARS1::state_washing(const Event& event)
{
    // A static variable is not really necessary
    static int washingTimeout = -1;

    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            // Open the feed and venting valves for washing time
            modules.get<Actuators>()->openServoAtomic(
                ServosList::VENTING_VALVE,
                Config::TARS::TARS_WASHING_OPENING_TIME +
                    Config::TARS::TARS_WASHING_TIME_DELAY);

            // Sleep to avoid actuators turn at the same time
            Thread::sleep(2 * Config::TARS::TARS_WASHING_TIME_DELAY);

            modules.get<Actuators>()->openServoAtomic(
                ServosList::FILLING_VALVE,
                Config::TARS::TARS_WASHING_OPENING_TIME);

            // After double the washing time it translates to the refueling
            washingTimeout = EventBroker::getInstance().postDelayed(
                TARS_WASHING_DONE, TOPIC_TARS,
                Config::TARS::TARS_WASHING_OPENING_TIME * 2);

            return logStatus(TARS1State::WASHING);
        }
        case TARS_WASHING_DONE:
        {
            // Open the filling valve for a long time
            modules.get<Actuators>()->openServoAtomic(
                ServosList::FILLING_VALVE,
                Config::TARS::TARS_FILLING_OPENING_TIME);

            // Wait for PRESSURE STABILIZE WAIT TIME
            Thread::sleep(Config::TARS::TARS_PRESSURE_STABILIZE_WAIT_TIME);

            return transition(&TARS1::state_refueling);
        }
        case MOTOR_MANUAL_ACTION:
        {
            // Delete the post delayed event
            EventBroker::getInstance().removeDelayed(washingTimeout);
            return transition(&TARS1::state_ready);
        }
        case TMTC_ARM:
        case MOTOR_STOP_TARS:
        case MOTOR_IGNITION:
        case MOTOR_START_TARS:
        {
            // Close all the valves
            modules.get<Actuators>()->closeAllServo();
            // Delete the post delayed event
            EventBroker::getInstance().removeDelayed(washingTimeout);
            // If another type of event is thrown the FSM returns to ready state
            return transition(&TARS1::state_ready);
        }
    }
}

void TARS1::state_refueling(const Event& event)
{
    // A static variable is not really necessary
    static int pressureStabilize = -1;

    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case TARS_PRESSURE_STABILIZED:
        case EV_ENTRY:
        {
            // Check if the algorithm can fill more
            if (isMaximumMass())
            {
                if (massStableCounter >=
                    Config::TARS::TARS_NUMBER_MASS_STABLE_ITERATIONS)
                {
                    massStableCounter = 0;
                    return EventBroker::getInstance().post(TARS_FILLING_DONE,
                                                           TOPIC_TARS);
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

            // Open the venting and wait for the pressure to stabilize
            modules.get<Actuators>()->toggleServo(ServosList::VENTING_VALVE);
            pressureStabilize = EventBroker::getInstance().postDelayed(
                TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                modules.get<Actuators>()->getTiming(ServosList::VENTING_VALVE) +
                    Config::TARS::TARS_PRESSURE_STABILIZE_WAIT_TIME * 5);

            return logStatus(TARS1State::REFUELING);
        }
        case TARS_CHECK_PRESSURE_STABILIZE:
        {
            if (!isPressureStable())
            {
                // Iterate until the pressure is stable
                pressureStabilize = EventBroker::getInstance().postDelayed(
                    TARS_CHECK_PRESSURE_STABILIZE, TOPIC_TARS,
                    Config::TARS::TARS_PRESSURE_STABILIZE_WAIT_TIME);

                // Update the previous pressure sampling
                {
                    Lock<FastMutex> lock(mutex);
                    previousPressure = samples.pressure;
                }

                return;
            }

            Thread::sleep(Config::TARS::TARS_PRESSURE_STABILIZE_WAIT_TIME);

            // Update mass at venting
            {
                Lock<FastMutex> lock(mutex);
                previousMassAtVenting = massAtVenting;
                massAtVenting         = samples.mass;
            }

            return EventBroker::getInstance().post(TARS_PRESSURE_STABILIZED,
                                                   TOPIC_TARS);
        }
        case TARS_FILLING_DONE:
        {
            EventBroker::getInstance().removeDelayed(pressureStabilize);
            // Close all the valves and stop the algorithm
            modules.get<Actuators>()->closeAllServo();

            return transition(&TARS1::state_ready);
        }
        case MOTOR_MANUAL_ACTION:
        {
            EventBroker::getInstance().removeDelayed(pressureStabilize);
            return transition(&TARS1::state_ready);
        }
        case TMTC_ARM:
        case MOTOR_STOP_TARS:
        case MOTOR_IGNITION:
        case MOTOR_START_TARS:
        {
            EventBroker::getInstance().removeDelayed(pressureStabilize);
            // Close all the valves
            modules.get<Actuators>()->closeAllServo();
            // If another type of event is thrown the FSM returns to ready state
            return transition(&TARS1::state_ready);
        }
    }
}

void TARS1::logStatus(TARS1State state)
{
    // Set the current state to be logged by the sample thread
    status.state = state;
}

TARS1Status TARS1::getStatus()
{
    PauseKernelLock lock;
    return status;
}

void TARS1::sample()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Sample the sensors and update the averages
    {
        Lock<FastMutex> lock(mutex);

        float pressure = modules.get<Sensors>()->getTankBottomPress().pressure;
        float mass     = modules.get<Sensors>()->getTankWeight().load;

        filteredData.pressure.add(pressure);
        filteredData.mass.add(mass);

        sampleCounter++;

        // When the number of samples reaches the template buffer number
        if (filteredData.pressure.isReady())
        {
            samples.pressure = filteredData.pressure.getMedian();
            samples.mass     = filteredData.mass.getMedian();
            sampleCounter    = 0;

            // Log the TARS state
            status.timestamp = TimestampTimer::getTimestamp();
            status.pressure  = samples.pressure;
            status.mass      = samples.mass;

            Logger::getInstance().log(status);
        }
    }
}

bool TARS1::isMaximumMass()
{
    {
        Lock<FastMutex> lock(mutex);
        return (abs(massAtVenting - previousMassAtVenting) <
                Config::TARS::TARS_MASS_TOLERANCE);
    }
}

bool TARS1::isPressureStable()
{
    {
        Lock<FastMutex> lock(mutex);
        return (abs(samples.pressure - previousPressure) <
                Config::TARS::TARS_PRESSURE_TOLERANCE);
    }
}
}  // namespace RIG