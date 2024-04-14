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
#include <RIGv2/Configs/SchedulerConfig.h>
#include <RIGv2/Sensors/Sensors.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

#include "GroundModeManagerData.h"

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

GroundModeManager::GroundModeManager()
    : FSM(&GroundModeManager::state_idle, miosix::STACK_DEFAULT_FOR_PTHREAD,
          GMM_PRIORITY)
{
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
}

GroundModeManagerState GroundModeManager::getState()
{
    if(testState(&GroundModeManager::state_init_err)) {
        return GMM_STATE_INIT_ERR;
    } else if(testState(&GroundModeManager::state_disarmed)) {
        return GMM_STATE_DISARMED;
    } else if(testState(&GroundModeManager::state_armed)) {
        return GMM_STATE_ARMED;
    } else if(testState(&GroundModeManager::state_igniting)) {
        return GMM_STATE_IGNITING;
    } else {
        return GMM_STATE_IDLE;
    }
}

void GroundModeManager::setIgnitionTime(uint32_t time) { ignitionTime = time; }

void GroundModeManager::state_idle(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            break;
        }

        case TMTC_RESET_BOARD:
        {
            reboot();
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
            logStatus();
            break;
        }

        case TMTC_RESET_BOARD:
        {
            reboot();
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
            modules.get<Actuators>()->armLightOff();
            logStatus();
            break;
        }

        case TMTC_RESET_BOARD:
        {
            reboot();
            break;
        }

        case TMTC_OPEN_NITROGEN:
        {
            modules.get<Actuators>()->openNitrogen();
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
            modules.get<Actuators>()->armLightOn();
            modules.get<Actuators>()->closeAllServos();
            modules.get<Actuators>()->closeNitrogen();
            logStatus();
            break;
        }

        case TMTC_OPEN_NITROGEN:
        {
            modules.get<Actuators>()->openNitrogen();

            // Nitrogen causes automatic disarm
            transition(&GroundModeManager::state_disarmed);
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
            modules.get<Actuators>()->igniterOn();

            // Send event to open the oxidant
            openOxidantDelayEventId = EventBroker::getInstance().postDelayed(
                MOTOR_OPEN_OXIDANT, TOPIC_MOTOR, ignitionTime);

            logStatus();
            break;
        }

        case EV_EXIT:
        {
            // Disable MOTOR_OPEN_OXIDANT event
            EventBroker::getInstance().removeDelayed(openOxidantDelayEventId);
        }

        case MOTOR_OPEN_OXIDANT:
        {
            // Stop ignition
            modules.get<Actuators>()->igniterOff();
            modules.get<Actuators>()->openServo(ServosList::MAIN_VALVE);
            break;
        }

        case TMTC_OPEN_NITROGEN:
        {
            // Stop ignition and close all servos
            modules.get<Actuators>()->igniterOff();
            modules.get<Actuators>()->closeAllServos();

            // Open nitrogen
            modules.get<Actuators>()->openNitrogen();

            transition(&GroundModeManager::state_disarmed);
            break;
        }

        case TMTC_DISARM:  // Abort signal
        case MOTOR_CLOSE_FEED_VALVE:
        {
            // Stop ignition and close all servos
            modules.get<Actuators>()->igniterOff();
            modules.get<Actuators>()->closeAllServos();

            transition(&GroundModeManager::state_disarmed);
            break;
        }
    }
}

void GroundModeManager::logStatus()
{
    GroundModeManagerData data = {TimestampTimer::getTimestamp(), getState()};
    sdLogger.log(data);
}