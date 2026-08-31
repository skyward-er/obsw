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

#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace RIGv3;
using namespace Boardcore;
using namespace Common;
using namespace miosix;

GroundModeManager::GroundModeManager()
    : HSM(&GroundModeManager::state_idle, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::groundModeManagerPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

GroundModeManagerState GroundModeManager::getState() { return state; }

void GroundModeManager::powerOnElectronics(bool umbilical, bool main,
                                           bool engine, bool cots)
{
    auto* expander = &getModule<GpioExpander>()->getExpander();

    expander->setPinValue(Config::GpioExpander::RPO_UMBILICAL.getPort(),
                          Config::GpioExpander::RPO_UMBILICAL.getPin(),
                          umbilical);

    expander->setPinValue(Config::GpioExpander::RPO_MAIN.getPort(),
                          Config::GpioExpander::RPO_MAIN.getPin(), main);

    expander->setPinValue(Config::GpioExpander::RPO_ENGINE.getPort(),
                          Config::GpioExpander::RPO_ENGINE.getPin(), engine);

    expander->setPinValue(Config::GpioExpander::RPO_COTS.getPort(),
                          Config::GpioExpander::RPO_COTS.getPin(), cots);

    EventBroker::getInstance().postDelayed(
        TMTC_RPO_CLOCK_ON, TOPIC_TMTC, Config::GroundModeManager::RPO_ON_DELAY);
}

State GroundModeManager::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GroundModeManagerState::IDLE);
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

State GroundModeManager::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GroundModeManagerState::INIT);
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

State GroundModeManager::state_init_error(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GroundModeManagerState::INIT_ERR);
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

State GroundModeManager::state_disarmed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GroundModeManagerState::DISARMED);
            getModule<Actuators>()->armLightOff();
            getModule<Registry>()->disarm();
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::DISARM);
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

        case TMTC_ARM:
        {
            return transition(&GroundModeManager::state_armed);
        }

        case TMTC_CALIBRATE:
        {
            getModule<Sensors>()->calibrateLoadcells();
            getModule<Sensors>()->calibrateEncoders();

            getModule<CanHandler>()->sendEvent(CanConfig::EventId::CALIBRATE);
            return HANDLED;
        }

        case MOTOR_STOP_TARS:
        {
            EventBroker::getInstance().post(MOTOR_STOP_TARS, TOPIC_TARS);
            return HANDLED;
        }

        case MOTOR_START_TARS1:
        {
            // Stop any running TARS
            EventBroker::getInstance().post(MOTOR_STOP_TARS, TOPIC_TARS);
            EventBroker::getInstance().post(MOTOR_START_TARS1, TOPIC_TARS);
            return HANDLED;
        }

        case MOTOR_START_TARS3:
        {
            // Stop any running TARS
            EventBroker::getInstance().post(MOTOR_STOP_TARS, TOPIC_TARS);
            EventBroker::getInstance().post(MOTOR_START_TARS3, TOPIC_TARS);
            return HANDLED;
        }

        case TMTC_RPO_CLOCK_ON:
        {
            getModule<GpioExpander>()->getExpander().setPinValue(
                Config::GpioExpander::RPO_CLOCK.getPort(),
                Config::GpioExpander::RPO_CLOCK.getPin(), true);

            EventBroker::getInstance().postDelayed(
                TMTC_RPO_CLOCK_OFF, TOPIC_TMTC,
                Config::GroundModeManager::RPO_OFF_DELAY);

            return HANDLED;
        }

        case TMTC_RPO_CLOCK_OFF:
        {
            getModule<GpioExpander>()->getExpander().setPinValue(
                Config::GpioExpander::RPO_CLOCK.getPort(),
                Config::GpioExpander::RPO_CLOCK.getPin(), false);

            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State GroundModeManager::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GroundModeManagerState::ARMED);

            getModule<Registry>()->arm();
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::ARM);

            getModule<Actuators>()->closeAllValves();
            EventBroker::getInstance().post(MOTOR_STOP_TARS, TOPIC_TARS);

            // Power on the arm light last to indicate that the system is armed
            getModule<Actuators>()->armLightOn();

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

        case TMTC_DISARM:
        {
            EventBroker::getInstance().post(FIRING_SEQUENCE_ABORT,
                                            TOPIC_FIRING_SEQUENCE);
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::DISARM);
            return transition(&GroundModeManager::state_disarmed);
        }

        case MOTOR_IGNITION:
        {
            // If the motor is connected do not interfere with the firing
            // sequence
            if (getModule<MotorStatus>()->connected())
            {
                getModule<CanHandler>()->sendEvent(
                    CanConfig::EventId::IGNITION);
            }
            else
            {
                EventBroker::getInstance().post(FIRING_SEQUENCE_START,
                                                TOPIC_FIRING_SEQUENCE);
            }

            return transition(&GroundModeManager::state_firing);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State GroundModeManager::state_firing(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(GroundModeManagerState::FIRING);
            return HANDLED;
        }

        case EV_INIT:
        {
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

        case TMTC_DISARM:  // Abort signal
        {
            EventBroker::getInstance().post(FIRING_SEQUENCE_ABORT,
                                            TOPIC_FIRING_SEQUENCE);

            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::ENGINE_SHUTDOWN);
            return transition(&GroundModeManager::state_disarmed);
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
