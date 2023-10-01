/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <RIG/CanHandler/CanHandler.h>
#include <RIG/Configs/GroundModeManagerConfig.h>
#include <RIG/Sensors/Sensors.h>
#include <RIG/StateMachines/GroundModeManager/GroundModeManager.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace Common;

namespace RIG
{
GroundModeManager::GroundModeManager()
    : FSM(&GroundModeManager::state_idle),
      ignitionTime(Config::Ignition::DEFAULT_IGNITION_WAITING_TIME)
{
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

void GroundModeManager::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(GroundModeManagerState::IDLE);
        }
        case FMM_INIT_OK:
        {
            return transition(&GroundModeManager::state_ready);
        }
    }
}

void GroundModeManager::state_ready(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            // Send an DISARM CAN event
            modules.get<CanHandler>()->sendEvent(
                Common::CanConfig::EventId::DISARM);

            // Disarm the relays
            miosix::relays::ignition::high();
            miosix::relays::nitrogen::high();
            miosix::relays::ledLamp::high();

            return logStatus(GroundModeManagerState::READY);
        }
        case TMTC_ARM:
        {
            return transition(&GroundModeManager::state_armed);
        }
        case TMTC_CALIBRATE:
        {
            // Send CAN event
            modules.get<CanHandler>()->sendEvent(
                Common::CanConfig::EventId::CALIBRATE);

            // Blocking (but not too complex) operation of calibration
            modules.get<Sensors>()->calibrate();
            break;
        }
    }
}

void GroundModeManager::state_armed(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            // Send an ARM CAN event
            modules.get<CanHandler>()->sendEvent(
                Common::CanConfig::EventId::ARM);

            // Turn on the light
            miosix::relays::ledLamp::low();

            // Expire all servos timings
            modules.get<Actuators>()->closeAllServo();
            return logStatus(GroundModeManagerState::ARMED);
        }
        case TMTC_DISARM:
        {
            // Turn off the light
            miosix::relays::ledLamp::high();

            return transition(&GroundModeManager::state_ready);
        }
        case MOTOR_IGNITION:
        {
            return transition(&GroundModeManager::state_igniting);
        }
    }
}

void GroundModeManager::state_igniting(const Event& event)
{
    // A static variable is not really necessary
    static int oxidantTimeoutEventId = -1;

    ModuleManager& modules = ModuleManager::getInstance();

    switch (event)
    {
        case EV_ENTRY:
        {
            // Fire the igniter
            miosix::relays::ignition::low();

            // Set the delayed event for oxidant
            oxidantTimeoutEventId = EventBroker::getInstance().postDelayed(
                MOTOR_OPEN_OXIDANT, TOPIC_MOTOR, ignitionTime);

            return logStatus(GroundModeManagerState::IGNITING);
        }
        case MOTOR_OPEN_OXIDANT:
        {
            // Shut down the igniter
            miosix::relays::ignition::high();

            modules.get<Actuators>()->toggleServo(ServosList::MAIN_VALVE);
            break;
        }
        case MOTOR_CLOSE_FEED_VALVE:
        case TMTC_DISARM:
        {
            // Shut down the igniter
            miosix::relays::ignition::high();

            // Close all the valves
            modules.get<Actuators>()->closeAllServo();

            EventBroker::getInstance().removeDelayed(oxidantTimeoutEventId);

            for (uint32_t i = 0; i < Config::Ignition::NUMBER_NITROGEN_BUMPS;
                 i++)
            {
                Thread::sleep(Config::Ignition::DEAD_TIME_AFTER_BURN);
                // Open nitrogen
                miosix::relays::nitrogen::low();
                Thread::sleep(Config::Ignition::NITROGEN_OPENING_TIME);
                // Close nitrogen
                miosix::relays::nitrogen::high();
            }

            return transition(&GroundModeManager::state_ready);
        }
    }
}

void GroundModeManager::setIgnitionTime(uint32_t ignitionTime)
{
    this->ignitionTime = ignitionTime;
}

void GroundModeManager::logStatus(GroundModeManagerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

GroundModeManagerStatus GroundModeManager::getStatus()
{
    miosix::PauseKernelLock lock;
    return status;
}
}  // namespace RIG