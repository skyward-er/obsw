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

#include <Main/Configs/FMMConfig.h>
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
    EventBroker::getInstance().subscribe(this, TOPIC_MEA);
}

FlightModeManagerState FlightModeManager::getState() { return state; }

State FlightModeManager::state_on_ground(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FlightModeManagerState::ON_GROUND);
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
            updateAndLogStatus(FlightModeManagerState::INIT);
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
            updateAndLogStatus(FlightModeManagerState::INIT_ERROR);
            getModule<Actuators>()->setStatusErr();
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
            // This doesn't make much sense
            // getModule<CanHandler>()->sendEvent(CanConfig::EventId::FORCE_INIT);
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
            updateAndLogStatus(FlightModeManagerState::INIT_DONE);
            getModule<Actuators>()->setStatusOk();
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
            updateAndLogStatus(FlightModeManagerState::CALIBRATE_SENSORS);

            // Wait a bit before calibrating
            Thread::sleep(100);

            getModule<Sensors>()->calibrate();
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
            updateAndLogStatus(FlightModeManagerState::CALIBRATE_ALGORITHMS);

            // Reset readiness status
            nasReady = false;
            adaReady = false;

            // Wait a bit after sensor calibration
            Thread::sleep(100);

            // First calibrate the reference
            getModule<AlgoReference>()->calibrate();

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
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(FlightModeManagerState::DISARMED);

            getModule<Actuators>()->camOff();
            getModule<Actuators>()->setBuzzerOff();

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
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::CALIBRATE);
            return transition(&FlightModeManager::state_calibrate_sensors);
        }
        case TMTC_ENTER_TEST_MODE:
        {
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::ENTER_TEST_MODE);
            return transition(&FlightModeManager::state_test_mode);
        }
        case TMTC_ARM:
        {
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::ARM);
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
            updateAndLogStatus(FlightModeManagerState::TEST_MODE);

            // Reset all stats
            getModule<StatsRecorder>()->reset();

            EventBroker::getInstance().post(ADA_FORCE_START, TOPIC_ADA);
            EventBroker::getInstance().post(NAS_FORCE_START, TOPIC_NAS);
            EventBroker::getInstance().post(MEA_FORCE_START, TOPIC_MEA);
            getModule<Sensors>()->resetMagCalibrator();
            getModule<Sensors>()->enableMagCalibrator();
            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().post(ADA_FORCE_STOP, TOPIC_ADA);
            EventBroker::getInstance().post(NAS_FORCE_STOP, TOPIC_NAS);
            EventBroker::getInstance().post(MEA_FORCE_STOP, TOPIC_MEA);
            getModule<Sensors>()->disableMagCalibrator();
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
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::EXIT_TEST_MODE);
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
            updateAndLogStatus(FlightModeManagerState::ARMED);

            // Reset all stats
            getModule<StatsRecorder>()->reset();

            Logger::getInstance().stop();
            Logger::getInstance().start();
            Logger::getInstance().resetStats();

            getModule<Actuators>()->camOn();
            getModule<Actuators>()->setBuzzerArmed();

            EventBroker::getInstance().post(FLIGHT_ARMED, TOPIC_FLIGHT);

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
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::DISARM);
            return transition(&FlightModeManager::state_disarmed);
        }
        case TMTC_FORCE_LAUNCH:
        case FLIGHT_LAUNCH_PIN_DETACHED:
        {
            getModule<StatsRecorder>()->liftoffDetected(
                TimestampTimer::getTimestamp());
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::LIFTOFF);
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
            updateAndLogStatus(FlightModeManagerState::FLYING);

            getModule<Actuators>()->setBuzzerOff();

            // Post mission end timeout
            missionTimeoutEvent = EventBroker::getInstance().postDelayed(
                FMM_MISSION_TIMEOUT, TOPIC_FMM,
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
        case FMM_MISSION_TIMEOUT:
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
            updateAndLogStatus(FlightModeManagerState::POWERED_ASCENT);

            EventBroker::getInstance().post(FLIGHT_LIFTOFF, TOPIC_FLIGHT);

            // Safety engine shutdown
            engineShutdownEvent = EventBroker::getInstance().postDelayed(
                FMM_ENGINE_TIMEOUT, TOPIC_FMM,
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
        case FMM_ENGINE_TIMEOUT:
        {
            getModule<CanHandler>()->sendServoCloseCommand(
                ServosList::MAIN_VALVE);
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
            updateAndLogStatus(FlightModeManagerState::UNPOWERED_ASCENT);

            EventBroker::getInstance().post(FLIGHT_MOTOR_SHUTDOWN,
                                            TOPIC_FLIGHT);

            apogeeTimeoutEvent = EventBroker::getInstance().postDelayed(
                FMM_APOGEE_TIMEOUT, TOPIC_FMM,
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
        case ADA_APOGEE_DETECTED:
        case TMTC_FORCE_EXPULSION:
        case FMM_APOGEE_TIMEOUT:
        {
            EventBroker::getInstance().post(FLIGHT_APOGEE_DETECTED,
                                            TOPIC_FLIGHT);

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
            LOG_INFO(logger, "Expelled");

            updateAndLogStatus(FlightModeManagerState::DROGUE_DESCENT);

            getModule<Actuators>()->openExpulsion();
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::APOGEE_DETECTED);

            EventBroker::getInstance().post(FLIGHT_DROGUE_DESCENT,
                                            TOPIC_FLIGHT);

            // Vent the tank
            getModule<CanHandler>()->sendServoOpenCommand(
                ServosList::VENTING_VALVE, 600000);

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
        case ADA_DEPLOY_ALTITUDE_DETECTED:
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
            updateAndLogStatus(FlightModeManagerState::TERMINAL_DESCENT);

            EventBroker::getInstance().post(FLIGHT_DPL_ALT_DETECTED,
                                            TOPIC_FLIGHT);

            getModule<Actuators>()->cutterOn();
            cutterTimeoutEvent = EventBroker::getInstance().postDelayed(
                FMM_CUTTER_TIMEOUT, TOPIC_FMM,
                Config::FlightModeManager::CUT_DURATION);

            return HANDLED;
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(cutterTimeoutEvent);

            // Make sure the cutters are off
            getModule<Actuators>()->cutterOff();
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

        case FMM_CUTTER_TIMEOUT:
        {
            getModule<Actuators>()->cutterOff();
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
            updateAndLogStatus(FlightModeManagerState::LANDED);

            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);
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