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

#include "FlightModeManager.h"

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/FlightModeManagerConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Main;
using namespace Common;
using namespace Boardcore;
using namespace miosix;

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_on_ground, STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::FMM_PRIORITY)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
}

FlightModeManagerState FlightModeManager::getState()
{
    if (testState(&FlightModeManager::state_on_ground))
    {
        return FlightModeManagerState::ON_GROUND;
    }
    else if (testState(&FlightModeManager::state_init))
    {
        return FlightModeManagerState::INIT;
    }
    else if (testState(&FlightModeManager::state_init_error))
    {
        return FlightModeManagerState::INIT_ERROR;
    }
    else if (testState(&FlightModeManager::state_init_done))
    {
        return FlightModeManagerState::INIT_DONE;
    }
    else if (testState(&FlightModeManager::state_calibrate_sensors))
    {
        return FlightModeManagerState::CALIBRATE_SENSORS;
    }
    else if (testState(&FlightModeManager::state_calibrate_algorithms))
    {
        return FlightModeManagerState::CALIBRATE_ALGORITHMS;
    }
    else if (testState(&FlightModeManager::state_disarmed))
    {
        return FlightModeManagerState::DISARMED;
    }
    else if (testState(&FlightModeManager::state_test_mode))
    {
        return FlightModeManagerState::TEST_MODE;
    }
    else if (testState(&FlightModeManager::state_armed))
    {
        return FlightModeManagerState::ARMED;
    }
    else if (testState(&FlightModeManager::state_flying))
    {
        return FlightModeManagerState::FLYING;
    }
    else if (testState(&FlightModeManager::state_powered_ascent))
    {
        return FlightModeManagerState::POWERED_ASCENT;
    }
    else if (testState(&FlightModeManager::state_unpowered_ascent))
    {
        return FlightModeManagerState::UNPOWERED_ASCENT;
    }
    else if (testState(&FlightModeManager::state_drogue_descent))
    {
        return FlightModeManagerState::DROGUE_DESCENT;
    }
    else if (testState(&FlightModeManager::state_terminal_descent))
    {
        return FlightModeManagerState::TERMINAL_DESCENT;
    }
    else if (testState(&FlightModeManager::state_landed))
    {
        return FlightModeManagerState::LANDED;
    }
    else
    {
        return FlightModeManagerState::INVALID;
    }
}

State FlightModeManager::state_on_ground(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_top);
        }
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_init);
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

State FlightModeManager::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case FMM_INIT_ERROR:
        {
            return transition(&FlightModeManager::state_init_error);
        }
        case FMM_INIT_OK:
        {
            return transition(&FlightModeManager::state_init_done);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_init_error(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case CAN_FORCE_INIT:
        {
            return transition(&FlightModeManager::state_init_done);
        }
        case TMTC_FORCE_INIT:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_init_done);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_init_done(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            EventBroker::getInstance().post(FMM_CALIBRATE, TOPIC_FMM);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case FMM_CALIBRATE:
        {
            return transition(&FlightModeManager::state_calibrate_sensors);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_calibrate_sensors(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            // TODO(davide.mor): Calibrate sensors
            EventBroker::getInstance().post(FMM_SENSORS_CAL_DONE, TOPIC_FMM);
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case FMM_SENSORS_CAL_DONE:
        {
            return transition(&FlightModeManager::state_calibrate_algorithms);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_calibrate_algorithms(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            // Reset readiness status
            nasReady = false;
            adaReady = false;

            // Post the calibration events
            EventBroker::getInstance().post(NAS_CALIBRATE, TOPIC_NAS);
            EventBroker::getInstance().post(ADA_CALIBRATE, TOPIC_ADA);

            // Quick hack to make the state machine go forward
            EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
            EventBroker::getInstance().post(ADA_READY, TOPIC_ADA);

            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case NAS_READY:
        {
            nasReady = true;
            if (adaReady && nasReady)
            {
                return transition(&FlightModeManager::state_disarmed);
            }
            else
            {
                return HANDLED;
            }
        }
        case ADA_READY:
        {
            adaReady = true;
            if (adaReady && nasReady)
            {
                return transition(&FlightModeManager::state_disarmed);
            }
            else
            {
                return HANDLED;
            }
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_disarmed(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            modules.get<Actuators>()->camOff();
            modules.get<Actuators>()->setBuzzerOff();

            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case CAN_CALIBRATE:
        {
            return transition(&FlightModeManager::state_calibrate_sensors);
        }
        case CAN_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::state_test_mode);
        }
        case CAN_ARM:
        {
            return transition(&FlightModeManager::state_armed);
        }
        case TMTC_CALIBRATE:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_calibrate_sensors);
        }
        case TMTC_ENTER_TEST_MODE:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_test_mode);
        }
        case TMTC_ARM:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_armed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_test_mode(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();
            // TODO(davide.mor): Start algorithms
            return HANDLED;
        }
        case EV_EXIT:
        {
            // TODO(davide.mor): Stop algorithms
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case CAN_EXIT_TEST_MODE:
        {
            return transition(&FlightModeManager::state_disarmed);
        }
        case TMTC_START_RECORDING:
        {
            modules.get<Actuators>()->camOn();
            return HANDLED;
        }
        case TMTC_STOP_RECORDING:
        {
            modules.get<Actuators>()->camOff();
            return HANDLED;
        }
        case TMTC_EXIT_TEST_MODE:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_disarmed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_armed(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            Logger::getInstance().stop();
            Logger::getInstance().start();

            modules.get<Actuators>()->camOn();
            modules.get<Actuators>()->setBuzzerArmed();

            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_top);
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case CAN_DISARM:
        {
            return transition(&FlightModeManager::state_disarmed);
        }
        case TMTC_DISARM:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_disarmed);
        }
        case TMTC_FORCE_LAUNCH:
        case FLIGHT_LAUNCH_PIN_DETACHED:
        {
            // TODO(davide.mor): Also send this via CanBus
            return transition(&FlightModeManager::state_flying);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_flying(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            modules.get<Actuators>()->setBuzzerOff();

            // Post mission end timeout
            missionTimeoutEvent = EventBroker::getInstance().postDelayed(
                FLIGHT_MISSION_TIMEOUT, TOPIC_FLIGHT,
                Config::FlightModeManager::MISSION_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(missionTimeoutEvent);
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_top);
        }
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_powered_ascent);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&FlightModeManager::state_landed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_powered_ascent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            // Safety engine shutdown
            engineShutdownEvent = EventBroker::getInstance().postDelayed(
                MOTOR_CLOSE_FEED_VALVE, TOPIC_FMM,
                Config::FlightModeManager::ENGINE_SHUTDOWN_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(engineShutdownEvent);
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_flying);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case MEA_SHUTDOWN_DETECTED:
        case MOTOR_CLOSE_FEED_VALVE:
        {
            // TODO(davide.mor): Actually shutdown the motor via CanBus

            return transition(&FlightModeManager::state_unpowered_ascent);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_unpowered_ascent(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            apogeeTimeoutEvent = EventBroker::getInstance().postDelayed(
                TMTC_FORCE_APOGEE, TOPIC_TMTC,
                Config::FlightModeManager::APOGEE_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_flying);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_FORCE_APOGEE:
        case ADA_APOGEE_DETECTED:
        {
            // TODO(davide.mor): Also send this via CanBus
            modules.get<Actuators>()->openExpulsion();

            return transition(&FlightModeManager::state_drogue_descent);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_drogue_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            // TODO(davide.mor): Perform venting?

            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_flying);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_FORCE_DEPLOYMENT:
        case ALTITUDE_TRIGGER_ALTITUDE_REACHED:
        {
            return transition(&FlightModeManager::state_terminal_descent);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_terminal_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            // TODO(davide.mor): Actuate cutters?

            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_flying);
        }
        case EV_INIT:
        {
            return HANDLED;
        }
        case TMTC_FORCE_LANDING:
        {
            return transition(&FlightModeManager::state_landed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_landed(const Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus();

            Logger::getInstance().stop();

            modules.get<Actuators>()->setBuzzerLand();

            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_top);
        }
        case EV_INIT:
        {
            return HANDLED;
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

void FlightModeManager::logStatus()
{
    FlightModeManagerStatus status;
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = getState();

    Logger::getInstance().log(status);
}