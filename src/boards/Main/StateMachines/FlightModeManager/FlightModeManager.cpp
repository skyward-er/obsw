/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Angelo Prete, Matteo Pignataro
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

#include <Main/Actuators/Actuators.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/Configs/FlightModeManagerConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>

#include "FlightModeManagerData.h"

using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Main::FMMConfig;

namespace Main
{
// (TODO) Add correctly CAN Events (without duping)

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_on_ground)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
    EventBroker::getInstance().subscribe(this, TOPIC_CAN);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_ALT);
    EventBroker::getInstance().subscribe(this, TOPIC_MEA);
}

FlightModeManagerStatus FlightModeManager::getStatus()
{
    PauseKernelLock lock;
    return status;
}

State FlightModeManager::state_on_ground(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::ON_GROUND);

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
            logStatus(FlightModeManagerState::INIT);
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
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::INIT_ERROR);
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
        case TMTC_FORCE_INIT:
        {
            if (event != CAN_FORCE_INIT)
            {
                modules.get<CanHandler>()->sendEvent(
                    CanConfig::EventId::FORCE_INIT);
            }
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
            logStatus(FlightModeManagerState::INIT_DONE);
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
            logStatus(FlightModeManagerState::CALIBRATE_SENSORS);

            ModuleManager::getInstance().get<Sensors>()->calibrate();
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
    static bool nasReady = false;
    static bool adaReady = false;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::CALIBRATE_ALGORITHMS);

            // Reset the readiness variables (not doing this could lead to
            // mis-calibration in future attempts)
            nasReady = false;
            adaReady = false;

            // Post the calibration events
            EventBroker::getInstance().post(NAS_CALIBRATE, TOPIC_NAS);
            EventBroker::getInstance().post(ADA_CALIBRATE, TOPIC_ADA);

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
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::DISARMED);

            // Stop eventual logging
            Logger::getInstance().stop();
            modules.get<Actuators>()->camOff();
            modules.get<Actuators>()->setBuzzerOff();
            EventBroker::getInstance().post(FLIGHT_DISARMED, TOPIC_FLIGHT);

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
        case TMTC_CALIBRATE:
        {
            if (event != CAN_CALIBRATE)
            {
                modules.get<CanHandler>()->sendEvent(
                    CanConfig::EventId::CALIBRATE);
            }
            return transition(&FlightModeManager::state_calibrate_sensors);
        }
        case CAN_ENTER_TEST_MODE:
        case TMTC_ENTER_TEST_MODE:
        {
            if (event != CAN_ENTER_TEST_MODE)
            {
                modules.get<CanHandler>()->sendEvent(
                    CanConfig::EventId::ENTER_TEST_MODE);
            }
            return transition(&FlightModeManager::state_test_mode);
        }
        case CAN_ARM:
        case TMTC_ARM:
        {
            if (event != CAN_ARM)
            {
                modules.get<CanHandler>()->sendEvent(CanConfig::EventId::ARM);
            }
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
            logStatus(FlightModeManagerState::TEST_MODE);

            Logger::getInstance().start();
            EventBroker::getInstance().post(NAS_FORCE_START, TOPIC_NAS);

            return HANDLED;
        }
        case EV_EXIT:
        {
            Logger::getInstance().stop();
            EventBroker::getInstance().post(NAS_FORCE_STOP, TOPIC_NAS);
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
        case CAN_EXIT_TEST_MODE:
        case TMTC_EXIT_TEST_MODE:
        {
            if (event != CAN_EXIT_TEST_MODE)
            {
                modules.get<CanHandler>()->sendEvent(
                    CanConfig::EventId::EXIT_TEST_MODE);
            }
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
            logStatus(FlightModeManagerState::ARMED);

            Logger::getInstance().start();
            modules.get<Actuators>()->camOn();
            modules.get<Actuators>()->setBuzzerArm();
            EventBroker::getInstance().post(FLIGHT_ARMED, TOPIC_FLIGHT);

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
        case CAN_DISARM:
        case TMTC_DISARM:
        {
            if (event != CAN_DISARM)
            {
                modules.get<CanHandler>()->sendEvent(
                    CanConfig::EventId::DISARM);
            }
            return transition(&FlightModeManager::state_disarmed);
        }
        case FLIGHT_LAUNCH_PIN_DETACHED:
        case TMTC_FORCE_LAUNCH:
        {
            modules.get<CanHandler>()->sendEvent(CanConfig::EventId::LIFTOFF);
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
    static uint16_t missionTimeoutEventId = 0;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::FLYING);

            EventBroker::getInstance().post(FLIGHT_LIFTOFF, TOPIC_FLIGHT);
            missionTimeoutEventId = EventBroker::getInstance().postDelayed(
                FLIGHT_MISSION_TIMEOUT, TOPIC_FLIGHT, MISSION_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(missionTimeoutEventId);
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
            EventBroker::getInstance().post(FLIGHT_LANDING_TIMEOUT,
                                            TOPIC_FLIGHT);
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
    static uint16_t engineShutdownTimeoutEventId = 0;
    ModuleManager& modules                       = ModuleManager::getInstance();

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::POWERED_ASCENT);

            // After a maximum time, the motor is considered shut down
            engineShutdownTimeoutEventId =
                EventBroker::getInstance().postDelayed(
                    MOTOR_CLOSE_FEED_VALVE, TOPIC_FMM, ENGINE_SHUTDOWN_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            // Shutdown via can
            modules.get<CanHandler>()->sendCanCommand(ServosList::MAIN_VALVE, 0,
                                                      ENGINE_SHUTDOWN_TIMEOUT);

            EventBroker::getInstance().removeDelayed(
                engineShutdownTimeoutEventId);

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
    static uint16_t apogeeTimeoutEventId = 0;
    ModuleManager& modules               = ModuleManager::getInstance();

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::UNPOWERED_ASCENT);
            EventBroker::getInstance().post(FLIGHT_MOTOR_SHUTDOWN,
                                            TOPIC_FLIGHT);
            apogeeTimeoutEventId = EventBroker::getInstance().postDelayed(
                TMTC_FORCE_APOGEE, TOPIC_TMTC, APOGEE_EVENT_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            modules.get<Actuators>()->setServoPosition(
                ServosList::EXPULSION_SERVO, 1);

            EventBroker::getInstance().removeDelayed(apogeeTimeoutEventId);
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
            modules.get<CanHandler>()->sendEvent(
                CanConfig::EventId::APOGEE_DETECTED);
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
    ModuleManager& modules = ModuleManager::getInstance();
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::DROGUE_DESCENT);
            EventBroker::getInstance().post(FLIGHT_APOGEE_DETECTED,
                                            TOPIC_FLIGHT);
            EventBroker::getInstance().post(FLIGHT_DROGUE_DESCENT,
                                            TOPIC_FLIGHT);

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
            logStatus(FlightModeManagerState::TERMINAL_DESCENT);
            EventBroker::getInstance().post(FLIGHT_DPL_ALT_DETECTED,
                                            TOPIC_FLIGHT);

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
            logStatus(FlightModeManagerState::LANDED);
            modules.get<Actuators>()->setBuzzerLand();
            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);

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

void FlightModeManager::logStatus(FlightModeManagerState state)
{
    {
        PauseKernelLock lock;
        status.timestamp = TimestampTimer::getTimestamp();
        status.state     = state;
    }

    Logger::getInstance().log(status);
}

}  // namespace Main