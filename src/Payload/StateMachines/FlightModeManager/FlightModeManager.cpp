/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Niccolò Betto
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

#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/BoardScheduler.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/Configs/FMMConfig.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
using namespace std::chrono;
namespace config = Payload::Config::FlightModeManager;

namespace Payload
{

void enterHilMode()
{
    PersistentVars::setHilMode(true);
    miosix::reboot();
}

void exitHilMode()
{
    // Reboot only if in HIL mode
    if (PersistentVars::getHilMode())
    {
        PersistentVars::setHilMode(false);
        miosix::reboot();
    }
}

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::OnGround, miosix::STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::flightModeManagerPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_CAN);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_ALT);
}

FlightModeManager::~FlightModeManager()
{
    EventBroker::getInstance().unsubscribe(this);
}

FlightModeManagerState FlightModeManager::getState() { return state; }

bool FlightModeManager::isTestMode() const
{
    return state == FlightModeManagerState::ON_GROUND_TEST_MODE;
}

bool FlightModeManager::referenceUpdateAllowed() const
{
    auto s = state.load();

    return s == FlightModeManagerState::ON_GROUND_DISARMED ||
           s == FlightModeManagerState::ON_GROUND_TEST_MODE;
}

bool FlightModeManager::servoMovesAllowed() const
{
    auto s = state.load();

    return s == FlightModeManagerState::ON_GROUND_TEST_MODE ||
           s == FlightModeManagerState::LANDED;
}

State FlightModeManager::OnGround(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
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
            return transition(&FlightModeManager::OnGroundInit);
        }

        case TMTC_START_LOGGING:
        {
            Logger::getInstance().start();
            return HANDLED;
        }

        case TMTC_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            return HANDLED;
        }

        case TMTC_EXIT_HIL_MODE:
        {
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::EXIT_HIL_MODE);
            miosix::Thread::sleep(1000);
            exitHilMode();
            return HANDLED;
        }
        case CAN_EXIT_HIL_MODE:
        {
            exitHilMode();
            return HANDLED;
        }

        case TMTC_RESET_BOARD:
        {
            Logger::getInstance().stop();
            miosix::reboot();
            __builtin_unreachable();
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundInit(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::ON_GROUND_INIT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_INIT_OK:
        {
            return transition(&FlightModeManager::OnGroundInitDone);
        }

        case FMM_INIT_ERROR:
        {
            return transition(&FlightModeManager::OnGroundInitError);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundInitError(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::ON_GROUND_INIT_ERROR);
            getModule<Actuators>()->setStatusError();
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_FORCE_INIT:
        case CAN_FORCE_INIT:
        {
            return transition(&FlightModeManager::OnGroundInitDone);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundInitDone(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::ON_GROUND_INIT_DONE);
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
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_CALIBRATE:
        {
            return transition(&FlightModeManager::OnGroundSensorCalibration);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundSensorCalibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::ON_GROUND_SENSOR_CALIBRATION);

            // Wait for sensors to stabilize before calibration
            // The first few LPS28DFW samples contain garbage data
            miosix::Thread::sleep(100);
            getModule<Sensors>()->calibrate();

            EventBroker::getInstance().post(FMM_ALGOS_CALIBRATE, TOPIC_FMM);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_ALGOS_CALIBRATE:
        {
            return transition(&FlightModeManager::OnGroundAlgorithmCalibration);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundAlgorithmCalibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(
                FlightModeManagerState::ON_GROUND_ALGORITHM_CALIBRATION);
            // Calibrate after a delay to allow calibrated sensors to stabilize
            EventBroker::getInstance().postDelayed(NAS_CALIBRATE, TOPIC_NAS,
                                                   100);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case NAS_READY:
        {
            EventBroker::getInstance().post(FMM_READY, TOPIC_FMM);
            return transition(&FlightModeManager::OnGroundDisarmed);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundDisarmed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::ON_GROUND_DISARMED);
            getModule<Actuators>()->setBuzzerOff();
            getModule<Actuators>()->cameraOff();
            EventBroker::getInstance().post(FLIGHT_DISARMED, TOPIC_FLIGHT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_ARM:
        {
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::ARM);

            [[fallthrough]];
        }
        case CAN_ARM:
        {
            return transition(&FlightModeManager::Armed);
        }

        case TMTC_ENTER_TEST_MODE:
        {
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::ENTER_TEST_MODE);

            [[fallthrough]];
        }
        case CAN_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::OnGroundTestMode);
        }

        case TMTC_CALIBRATE:
        {
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::CALIBRATE);

            [[fallthrough]];
        }
        case TMTC_SET_CALIBRATION_PRESSURE:
        case CAN_CALIBRATE:
        {
            return transition(&FlightModeManager::OnGroundSensorCalibration);
        }

        case TMTC_RESET_NAS:
        {
            EventBroker::getInstance().post(NAS_RESET, TOPIC_NAS);
            return HANDLED;
        }

        case TMTC_RESET_ADA:
        {
            EventBroker::getInstance().post(ADA_RESET, TOPIC_ADA);
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::OnGroundTestMode(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::ON_GROUND_TEST_MODE);
            EventBroker::getInstance().post(NAS_FORCE_START, TOPIC_NAS);
            getModule<Sensors>()->resetMagCalibrator();
            getModule<Sensors>()->enableMagCalibrator();
            return HANDLED;
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->cameraOff();
            EventBroker::getInstance().post(NAS_FORCE_STOP, TOPIC_NAS);
            getModule<Sensors>()->disableMagCalibrator();
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::OnGround);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_START_RECORDING:
        {
            getModule<Actuators>()->cameraOn();
            return HANDLED;
        }

        case TMTC_STOP_RECORDING:
        {
            getModule<Actuators>()->cameraOff();
            return HANDLED;
        }

        case TMTC_ENTER_HIL_MODE:
        {
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::ENTER_HIL_MODE);
            miosix::Thread::sleep(1000);
            enterHilMode();
            return HANDLED;
        }
        case CAN_ENTER_HIL_MODE:
        {
            enterHilMode();
            return HANDLED;
        }

        case TMTC_EXIT_TEST_MODE:
        {
            getModule<CanHandler>()->sendEvent(
                CanConfig::EventId::EXIT_TEST_MODE);

            [[fallthrough]];
        }
        case CAN_EXIT_TEST_MODE:
        {
            return transition(&FlightModeManager::OnGroundDisarmed);
        }

        case TMTC_RESET_NAS:
        {
            EventBroker::getInstance().post(NAS_RESET, TOPIC_NAS);
            return HANDLED;
        }

        case TMTC_RESET_ADA:
        {
            EventBroker::getInstance().post(ADA_RESET, TOPIC_ADA);
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::Armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // Start a new log file for the flight
            auto& logger = Logger::getInstance();
            logger.stop();
            logger.start();
            // Ignore errors that occurred while logger was being restarted
            logger.resetStats();

            updateState(FlightModeManagerState::ARMED);

            getModule<Actuators>()->setBuzzerArmed();
            getModule<Actuators>()->cameraOn();

            EventBroker::getInstance().post(FLIGHT_ARMED, TOPIC_FLIGHT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            getModule<Actuators>()->setBuzzerOff();
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

        case TMTC_DISARM:
        {
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::DISARM);

            [[fallthrough]];
        }
        case CAN_DISARM:
        {
            return transition(&FlightModeManager::OnGroundDisarmed);
        }

        case TMTC_FORCE_LAUNCH:
        {
            getModule<CanHandler>()->sendEvent(CanConfig::EventId::LIFTOFF);

            [[fallthrough]];
        }
        case CAN_LIFTOFF:
        case FLIGHT_LAUNCH_PIN_DETACHED:
        {
            getModule<FlightStatsRecorder>()->liftoffDetected(
                TimestampTimer::getTimestamp());
            return transition(&FlightModeManager::Flying);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::Flying(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            EventBroker::getInstance().postDelayed(
                FLIGHT_MISSION_TIMEOUT, TOPIC_FLIGHT,
                milliseconds{config::MISSION_TIMEOUT}.count());
            EventBroker::getInstance().postDelayed(
                FLIGHT_NC_DETACHED, TOPIC_FLIGHT,
                milliseconds{config::APOGEE_TIMEOUT}.count());

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
            return transition(&FlightModeManager::FlyingAscending);
        }

        case TMTC_FORCE_LANDING:
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&FlightModeManager::Landed);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::FlyingAscending(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::FLYING_ASCENDING);
            EventBroker::getInstance().post(FLIGHT_LIFTOFF, TOPIC_FLIGHT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::Flying);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case CAN_APOGEE_DETECTED:
        case FLIGHT_NC_DETACHED:
        case TMTC_FORCE_EXPULSION:
        {
            auto gps       = getModule<Sensors>()->getUBXGPSLastSample();
            auto nasState  = getModule<NASController>()->getNasState();
            float altitude = -nasState.d;

            getModule<FlightStatsRecorder>()->apogeeDetected(
                TimestampTimer::getTimestamp(), gps.latitude, gps.longitude,
                altitude);

            return transition(&FlightModeManager::FlyingDrogueDescent);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::FlyingDrogueDescent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::FLYING_DROGUE_DESCENT);
            getModule<AltitudeTrigger>()->enable();
            EventBroker::getInstance().post(FLIGHT_DROGUE_DESCENT,
                                            TOPIC_FLIGHT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            getModule<AltitudeTrigger>()->disable();
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::Flying);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case ALTITUDE_TRIGGER_ALTITUDE_REACHED:
        case TMTC_FORCE_DEPLOYMENT:
        {
            return transition(&FlightModeManager::FlyingWingDescent);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::FlyingWingDescent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::FLYING_WING_DESCENT);
            // Send the event to the WingController
            EventBroker::getInstance().post(FLIGHT_WING_DESCENT, TOPIC_FLIGHT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::Flying);
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

State FlightModeManager::Landed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::LANDED);

            getModule<Actuators>()->cameraOff();

            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);
            Logger::getInstance().stop();

            // Update the buzzer last to ensure all previous operations are done
            getModule<Actuators>()->setBuzzerLanded();
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
            Logger::getInstance().stop();
            miosix::reboot();
            __builtin_unreachable();
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

void FlightModeManager::updateState(FlightModeManagerState newState)
{
    state = newState;

    auto status = FlightModeManagerStatus{
        .timestamp = TimestampTimer::getTimestamp(),
        .state     = newState,
    };
    Logger::getInstance().log(status);
}

}  // namespace Payload
