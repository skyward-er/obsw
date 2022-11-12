/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro & Federico Mandelli
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

#include <ParafoilNew/Configs/FlightModeManagerConfig.h>
#include <ParafoilNew/Configs/WingConfig.h>
#include <ParafoilNew/Sensors/Sensors.h>
#include <ParafoilNew/Wing/WindEstimation.h>
#include <ParafoilNew/Wing/WingController.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>

using namespace miosix;
using namespace Boardcore;
using namespace Common;

namespace Payload
{

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
        case TMTC_START_RECORDING:
        {
            return HANDLED;
        }
        case TMTC_STOP_RECORDING:
        {
            return HANDLED;
        }
        case TMTC_RESET_BOARD:
        {
            Logger::getInstance().stop();
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
        case FMM_INIT_OK:
        {
            return transition(&FlightModeManager::state_sensors_calibration);
        }
        case FMM_INIT_ERROR:
        {
            logStatus(FlightModeManagerState::INIT_ERROR);
            EventBroker::getInstance().post(TMTC_RESET_BOARD, TOPIC_FLIGHT);
            return HANDLED;
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_sensors_calibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::SENSORS_CALIBRATION);

            Sensors::getInstance().calibrate();
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
            return transition(&FlightModeManager::state_algos_calibration);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_algos_calibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::ALGOS_CALIBRATION);

            EventBroker::getInstance().post(NAS_CALIBRATE, TOPIC_NAS);

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
            return transition(&FlightModeManager::state_flying);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

/*State FlightModeManager::state_test_mode(const Event& event) // we'll see if
needed
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::TEST_MODE);

            EventBroker::getInstance().post(NAS_FORCE_START, TOPIC_NAS);
            Logger::getInstance().start();

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().post(NAS_FORCE_STOP, TOPIC_NAS);
            Logger::getInstance().stop();

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
        case TMTC_EXIT_TEST_MODE:
        {
            return transition(&FlightModeManager::state_disarmed);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}*/

State FlightModeManager::state_flying(const Event& event)
{

    switch (event)
    {
        case EV_ENTRY:
        {
            static uint16_t missionTimeoutEventId = -1;
            missionTimeoutEventId =
                EventBroker::getInstance().postDelayed<MISSION_TIMEOUT>(
                    FLIGHT_MISSION_TIMEOUT, TOPIC_FLIGHT);
            return HANDLED;
        }
        case EV_EXIT:
        {
            // EventBroker::getInstance().removeDelayed(missionTimeoutEventId);
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::state_top);
        }
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_ascending);
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            WingController::getInstance().stop();
            return transition(&FlightModeManager::state_mission_ended);
        }
        case TMTC_FORCE_LANDING:
        {
            WingController::getInstance().stop();
            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);
            return HANDLED;
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_ascending(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::ASCENDING);
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
        case FLIGHT_WING_ALT_REACHED:
        case TMTC_FORCE_APOGEE:
        case TMTC_FORCE_EXPULSION:
        {
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
            logStatus(FlightModeManagerState::DROGUE_DESCENT);
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
        case FLIGHT_WING_ALT_REACHED:
        case TMTC_FORCE_MAIN:
        {
            return transition(&FlightModeManager::state_twirling);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_twirling(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::TWIRLING);

            static uint16_t timeoutEventId = -1;
            // start twirling and the 1st algorithm
            WingController::getInstance().stop();
            Actuators::getInstance().startTwirl();
            WindEstimation::getInstance()
                .startWindEstimationSchemeCalibration();
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
        case FLIGHT_WIND_PREDICTION:
        {
            Actuators::getInstance().stopTwirl();
            // now we begin the controlled part of the descent
            return transition(&FlightModeManager::state_controlled_descent);
        }
        case FLIGHT_WIND_PREDICTION_CALIBRATION:
        {
            WindEstimation::getInstance().stopWindEstimationSchemeCalibration();
            EventBroker::getInstance().postDelayed<FLIGHT_WIND_PREDICTION>(
                WingConfig::WIND_PREDICTION_TIMEOUT, TOPIC_FLIGHT);
            WindEstimation::getInstance().startWindEstimationScheme();
            return HANDLED;
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_controlled_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::CONTROLLED_DESCENT);

            WingController::getInstance().start();
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
        case FLIGHT_TWIRL:
        {
            WingController::getInstance().stop();
            return transition(&FlightModeManager::state_twirling);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_landed(const Event& event)
{
    static uint16_t landingTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::LANDED);
            landingTimeoutEventId =
                EventBroker::getInstance().postDelayed<LANDING_TIMEOUT>(
                    FLIGHT_LANDING_TIMEOUT, TOPIC_FLIGHT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(landingTimeoutEventId);
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
        case FLIGHT_LANDING_TIMEOUT:
        {
            return transition(&FlightModeManager::state_mission_ended);
        }
        case TMTC_RESET_BOARD:
        {
            Logger::getInstance().stop();
            reboot();
            return HANDLED;
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_mission_ended(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            Logger::getInstance().stop();

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
            reboot();
            return HANDLED;
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_on_ground)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
}

FlightModeManager::~FlightModeManager()
{
    EventBroker::getInstance().unsubscribe(this);
}

void FlightModeManager::logStatus(FlightModeManagerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

}  // namespace Payload
