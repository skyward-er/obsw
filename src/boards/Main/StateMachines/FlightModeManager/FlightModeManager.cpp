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
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
}

FlightModeManagerState FlightModeManager::getState()
{
    if (testState(&FlightModeManager::state_on_ground))
    {
        return FlightModeManagerState::FMM_STATE_ON_GROUND;
    }
    else if (testState(&FlightModeManager::state_init))
    {
        return FlightModeManagerState::FMM_STATE_INIT;
    }
    else if (testState(&FlightModeManager::state_init_error))
    {
        return FlightModeManagerState::FMM_STATE_INIT_ERROR;
    }
    else if (testState(&FlightModeManager::state_init_done))
    {
        return FlightModeManagerState::FMM_STATE_INIT_DONE;
    }
    else if (testState(&FlightModeManager::state_calibrate_sensors))
    {
        return FlightModeManagerState::FMM_STATE_CALIBRATE_SENSORS;
    }
    else if (testState(&FlightModeManager::state_calibrate_algorithms))
    {
        return FlightModeManagerState::FMM_STATE_CALIBRATE_ALGORITHMS;
    }
    else if (testState(&FlightModeManager::state_disarmed))
    {
        return FlightModeManagerState::FMM_STATE_DISARMED;
    }
    else if (testState(&FlightModeManager::state_test_mode))
    {
        return FlightModeManagerState::FMM_STATE_TEST_MODE;
    }
    else if (testState(&FlightModeManager::state_armed))
    {
        return FlightModeManagerState::FMM_STATE_ARMED;
    }
    else if (testState(&FlightModeManager::state_flying))
    {
        return FlightModeManagerState::FMM_STATE_FLYING;
    }
    else if (testState(&FlightModeManager::state_powered_ascent))
    {
        return FlightModeManagerState::FMM_STATE_POWERED_ASCENT;
    }
    else if (testState(&FlightModeManager::state_unpowered_ascent))
    {
        return FlightModeManagerState::FMM_STATE_UNPOWERED_ASCENT;
    }
    else if (testState(&FlightModeManager::state_drogue_descent))
    {
        return FlightModeManagerState::FMM_STATE_DROGUE_DESCENT;
    }
    else if (testState(&FlightModeManager::state_terminal_descent))
    {
        return FlightModeManagerState::FMM_STATE_TERMINAL_DESCENT;
    }
    else if (testState(&FlightModeManager::state_landed))
    {
        return FlightModeManagerState::FMM_STATE_LANDED;
    }
    else
    {
        return FlightModeManagerState::FMM_STATE_INVALID;
    }
}

State FlightModeManager::state_on_ground(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_ON_GROUND);
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
            updateAndLogStatus(FMM_STATE_INIT);
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
            updateAndLogStatus(FMM_STATE_INIT_ERROR);
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
            updateAndLogStatus(FMM_STATE_INIT_DONE);
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
            updateAndLogStatus(FMM_STATE_CALIBRATE_SENSORS);
            // TODO(davide.mor): Calibrate sensors
            Thread::sleep(2000);
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
            updateAndLogStatus(FMM_STATE_CALIBRATE_ALGORITHMS);

            // Reset readiness status
            nasReady = false;
            adaReady = false;

            // Post the calibration events
            EventBroker::getInstance().post(NAS_CALIBRATE, TOPIC_NAS);
            EventBroker::getInstance().post(ADA_CALIBRATE, TOPIC_ADA);

            // Quick hack to make the state machine go forward
            Thread::sleep(2000);
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
            if (adaReady)
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
            if (nasReady)
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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_DISARMED);

            getModule<Actuators>()->camOff();
            getModule<Actuators>()->setBuzzerOff();

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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_TEST_MODE);
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
            getModule<Actuators>()->camOn();
            return HANDLED;
        }
        case TMTC_STOP_RECORDING:
        {
            getModule<Actuators>()->camOff();
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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_ARMED);

            Logger::getInstance().stop();
            Logger::getInstance().start();

            getModule<Actuators>()->camOn();
            getModule<Actuators>()->setBuzzerArmed();

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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_FLYING);

            getModule<Actuators>()->setBuzzerOff();

            // Post mission end timeout
            missionTimeoutEvent = EventBroker::getInstance().postDelayed(
                FLIGHT_MISSION_TIMEOUT, TOPIC_FMM,
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
            updateAndLogStatus(FMM_STATE_POWERED_ASCENT);

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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_UNPOWERED_ASCENT);

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
            getModule<Actuators>()->openExpulsion();

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
            updateAndLogStatus(FMM_STATE_DROGUE_DESCENT);

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
            updateAndLogStatus(FMM_STATE_TERMINAL_DESCENT);

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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FMM_STATE_LANDED);

            Logger::getInstance().stop();

            getModule<Actuators>()->setBuzzerLand();

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

void FlightModeManager::updateAndLogStatus(FlightModeManagerState state)
{
    this->state = state;

    FlightModeManagerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}