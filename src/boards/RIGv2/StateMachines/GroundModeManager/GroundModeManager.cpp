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

#include <RIGv2/Configs/GmmConfig.h>
#include <RIGv2/Configs/SchedulerConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace RIGv2;
using namespace Boardcore;
using namespace Common;
using namespace miosix;

GroundModeManager::GroundModeManager()
    : HSM(&GroundModeManager::state_idle, STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::GMM_PRIORITY)
{
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

GroundModeManagerState GroundModeManager::getState() { return state; }

void GroundModeManager::setIgnitionTime(uint32_t time)
{
    getModule<Registry>()->setUnsafe(CONFIG_ID_IGNITION_TIME, time);
}

Boardcore::State GroundModeManager::state_idle(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_IDLE);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_top);
        }

        case EV_INIT:
        {
            return transition(&GroundModeManager::state_init);
        }

        case TMTC_RESET_BOARD:
        {
            reboot();
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_init(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_INIT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_idle);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_INIT_ERROR:
        {
            return transition(&GroundModeManager::state_init_error);
        }

        case FMM_INIT_OK:
        {
            return transition(&GroundModeManager::state_disarmed);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_init_error(
    const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_INIT_ERR);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_idle);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_FORCE_INIT:
        {
            return transition(&GroundModeManager::state_disarmed);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_disarmed(
    const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_DISARMED);
            getModule<Actuators>()->armLightOff();
            getModule<Registry>()->disarm();
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_idle);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_OPEN_NITROGEN:
        {
            getModule<Actuators>()->openNitrogen();
            return HANDLED;
        }

        case TMTC_ARM:
        {
            return transition(&GroundModeManager::state_armed);
        }

        case TMTC_CALIBRATE:
        {
            getModule<Sensors>()->calibrate();

            // TODO(davide.mor): Also send CAN command
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_armed(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_ARMED);
            getModule<Registry>()->arm();
            getModule<Actuators>()->armLightOn();
            getModule<Actuators>()->closeAllServos();
            getModule<Actuators>()->closeNitrogen();
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_top);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_OPEN_NITROGEN:
        {
            getModule<Actuators>()->openNitrogen();

            // Nitrogen causes automatic disarm
            return transition(&GroundModeManager::state_disarmed);
        }

        case TMTC_DISARM:
        {
            return transition(&GroundModeManager::state_disarmed);
        }

        case MOTOR_IGNITION:
        {
            return transition(&GroundModeManager::state_firing);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_firing(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_FIRING);
            return HANDLED;
        }

        case EV_EXIT:
        {
            // Stop ignition and close all servos
            getModule<Actuators>()->igniterOff();
            getModule<Actuators>()->closeAllServos();

            // Disable all events
            EventBroker::getInstance().removeDelayed(openOxidantDelayEventId);
            EventBroker::getInstance().removeDelayed(coolingDelayEventId);

            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_top);
        }

        case EV_INIT:
        {
            return transition(&GroundModeManager::state_igniting);
        }

        case TMTC_OPEN_NITROGEN:
        {
            // Open nitrogen
            getModule<Actuators>()->openNitrogen();

            return transition(&GroundModeManager::state_disarmed);
        }

        case MOTOR_COOLING_TIMEOUT:  // Normal firing end
        case TMTC_DISARM:            // Abort signal
        {
            // TODO(davide.mor): Set this to a sensible time
            getModule<Actuators>()->openNitrogenWithTime(
                Config::GroundModeManager::NITROGEN_TIME);

            return transition(&GroundModeManager::state_disarmed);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_igniting(
    const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_IGNITING);

            // Start ignition
            getModule<Actuators>()->igniterOn();

            uint32_t ignitionTime =
                getModule<Registry>()->getOrSetDefaultUnsafe(
                    CONFIG_ID_IGNITION_TIME,
                    Config::GroundModeManager::DEFAULT_IGNITION_WAITING_TIME);

            // Send event to open the oxidant
            openOxidantDelayEventId = EventBroker::getInstance().postDelayed(
                MOTOR_OPEN_OXIDANT, TOPIC_MOTOR, ignitionTime);

            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_firing);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case MOTOR_OPEN_OXIDANT:
        {
            getModule<Actuators>()->igniterOff();
            return transition(&GroundModeManager::state_oxidizer);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_oxidizer(
    const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_OXIDIZER);

            getModule<Actuators>()->openServo(ServosList::MAIN_VALVE);

            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_firing);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case MOTOR_CLOSE_FEED_VALVE:
        {
            return transition(&GroundModeManager::state_cooling);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

Boardcore::State GroundModeManager::state_cooling(const Boardcore::Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GMM_STATE_COOLING);

            coolingDelayEventId = EventBroker::getInstance().postDelayed(
                MOTOR_COOLING_TIMEOUT, TOPIC_MOTOR,
                Config::GroundModeManager::MOTOR_COOLING_TIME);

            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&GroundModeManager::state_firing);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

void GroundModeManager::updateAndLogStatus(GroundModeManagerState state)
{
    this->state = state;

    GroundModeManagerData data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
