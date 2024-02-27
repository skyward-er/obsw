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

#include "GroundModeManager.h"

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/Sensors/Sensors.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

void armLightOn() { relays::armLight::low(); }

void armLightOff() { relays::armLight::high(); }

void igniterOn() { relays::ignition::low(); }

void igniterOff() { relays::ignition::high(); }

GroundModeManager::GroundModeManager() : FSM(&GroundModeManager::state_idle)
{
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

bool GroundModeManager::isArmed()
{
    return testState(&GroundModeManager::state_armed);
}
bool GroundModeManager::isDisarmed()
{
    return testState(&GroundModeManager::state_disarmed);
}
bool GroundModeManager::isIgniting()
{
    return testState(&GroundModeManager::state_igniting);
}

void GroundModeManager::setIgnitionTime(uint32_t time) { ignitionTime = time; }

void GroundModeManager::state_idle(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(GroundModeManagerState::STATE_IDLE);
            break;
        }

        case FMM_INIT_ERROR:
        {
            transition(&GroundModeManager::state_init_err);
            break;
        }

        case FMM_INIT_OK:
        {
            transition(&GroundModeManager::state_disarmed);
            break;
        }
    }
}

void GroundModeManager::state_init_err(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(GroundModeManagerState::STATE_INIT_ERR);
            break;
        }

        case TMTC_FORCE_INIT:
        {
            transition(&GroundModeManager::state_disarmed);
            break;
        }
    }
}

void GroundModeManager::state_disarmed(const Boardcore::Event &event)
{
    ModuleManager &modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            armLightOff();
            logStatus(GroundModeManagerState::STATE_DISARMED);
            break;
        }

        case TMTC_ARM:
        {
            transition(&GroundModeManager::state_armed);
            break;
        }

        case TMTC_CALIBRATE:
        {
            modules.get<Sensors>()->calibrate();

            // TODO(davide.mor): Also send CAN command
            break;
        }
    }
}

void GroundModeManager::state_armed(const Boardcore::Event &event)
{
    ModuleManager &modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            armLightOn();
            modules.get<Actuators>()->closeAllServos();
            logStatus(GroundModeManagerState::STATE_ARMED);
            break;
        }

        case TMTC_DISARM:
        {
            transition(&GroundModeManager::state_disarmed);
            break;
        }

        case MOTOR_IGNITION:
        {
            transition(&GroundModeManager::state_igniting);
            break;
        }
    }
}

void GroundModeManager::state_igniting(const Boardcore::Event &event)
{
    ModuleManager &modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            // Start ignition
            igniterOn();

            // Send event to open the oxidant
            openOxidantDelayEventId = EventBroker::getInstance().postDelayed(
                MOTOR_OPEN_OXIDANT, TOPIC_MOTOR, ignitionTime);

            logStatus(GroundModeManagerState::STATE_IGNITING);
            break;
        }

        case MOTOR_OPEN_OXIDANT:
        {
            // Stop ignition
            igniterOff();
            modules.get<Actuators>()->openServo(ServosList::MAIN_VALVE);
            break;
        }

        case TMTC_DISARM:  // Abort signal
        case MOTOR_CLOSE_FEED_VALVE:
        {
            // Stop ignition
            igniterOff();

            // Close all of the valves
            modules.get<Actuators>()->closeAllServos();

            // Disable MOTOR_OPEN_OXIDANT event
            EventBroker::getInstance().removeDelayed(openOxidantDelayEventId);

            transition(&GroundModeManager::state_disarmed);
            break;
        }
    }
}

void GroundModeManager::logStatus(GroundModeManagerState newState)
{
    GroundModeManagerData data = {TimestampTimer::getTimestamp(), newState};
    sdLogger.log(data);
}