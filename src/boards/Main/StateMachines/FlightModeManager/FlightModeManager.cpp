/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
#include <Main/CanHandler/CanHandler.h>
#include <Main/Configs/FlightModeManagerConfig.h>
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/Sensors/Sensors.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>

using namespace miosix;
using namespace Boardcore;
using namespace Common;

namespace Main
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
        case TMTC_RESET_BOARD:
        {
            CanHandler::getInstance().sendCamOffCommand();
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
            return transition(&FlightModeManager::state_init_error);
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
            logStatus(FlightModeManagerState::INIT_ERROR);
            Actuators::getInstance().buzzerError();
            EventBroker::getInstance().post(FLIGHT_ERROR_DETECTED,
                                            TOPIC_FLIGHT);
            return HANDLED;
        }
        case EV_EXIT:
        {
            Actuators::getInstance().buzzerOff();
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
        case TMTC_FORCE_INIT:
        {
            return transition(&FlightModeManager::state_sensors_calibration);
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
    static bool nasReady = false;
    static bool adaReady = false;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::ALGOS_CALIBRATION);

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
                return transition(&FlightModeManager::state_disarmed);
            else
                return HANDLED;
        }
        case ADA_READY:
        {
            adaReady = true;

            if (nasReady)
                return transition(&FlightModeManager::state_disarmed);
            else
                return HANDLED;
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
            logStatus(FlightModeManagerState::DISARMED);

            Actuators::getInstance().buzzerDisarmed();
            Logger::getInstance().stop();
            EventBroker::getInstance().post(FLIGHT_DISARMED, TOPIC_FLIGHT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            Actuators::getInstance().buzzerOff();
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
        case TMTC_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::state_test_mode);
        }
        case TMTC_CALIBRATE:
        {
            return transition(&FlightModeManager::state_sensors_calibration);
        }
        case TMTC_ARM:
        {
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
            logStatus(FlightModeManagerState::TEST_MODE);

            EventBroker::getInstance().post(ADA_FORCE_START, TOPIC_NAS);
            EventBroker::getInstance().post(NAS_FORCE_START, TOPIC_NAS);
            Logger::getInstance().start();

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().post(ADA_FORCE_STOP, TOPIC_NAS);
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
}

State FlightModeManager::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::ARMED);

            Actuators::getInstance().buzzerArmed();
            Logger::getInstance().start();
            EventBroker::getInstance().post(FLIGHT_ARMED, TOPIC_FLIGHT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            Actuators::getInstance().buzzerOff();
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
            return transition(&FlightModeManager::state_disarmed);
        }
        case FLIGHT_LAUNCH_PIN_DETACHED:
        case TMTC_FORCE_LAUNCH:
        {
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
    static uint16_t missionTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            EventBroker::getInstance().post(FLIGHT_LIFTOFF, TOPIC_FLIGHT);
            missionTimeoutEventId =
                EventBroker::getInstance().postDelayed<MISSION_TIMEOUT>(
                    FLIGHT_MISSION_TIMEOUT, TOPIC_FLIGHT);
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
            return transition(&FlightModeManager::state_ascending);
        }
        case TMTC_FORCE_EXPULSION:
        {
            EventBroker::getInstance().post(DPL_NC_OPEN, TOPIC_DPL);
            return HANDLED;
        }
        case TMTC_FORCE_MAIN:
        {
            EventBroker::getInstance().post(DPL_CUT_DROGUE, TOPIC_DPL);
            return HANDLED;
        }
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&FlightModeManager::state_mission_ended);
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
        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_NC_DETACHED:
        case TMTC_FORCE_APOGEE:
        case TMTC_FORCE_EXPULSION:
        {
            FlightStatsRecorder::getInstance().setApogee(
                Sensors::getInstance().getUbxGpsLastSample());
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

            EventBroker::getInstance().post(DPL_NC_OPEN, TOPIC_DPL);

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
        case FLIGHT_DPL_ALT_DETECTED:
        case TMTC_FORCE_MAIN:
        {
            return transition(&FlightModeManager::state_terminal_descent);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_flying);
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

            EventBroker::getInstance().post(DPL_CUT_DROGUE, TOPIC_DPL);

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
            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);
            return HANDLED;
        }
        case FLIGHT_LANDING_DETECTED:
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
    static uint16_t landingTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::LANDED);
            landingTimeoutEventId =
                EventBroker::getInstance().postDelayed<LANDING_TIMEOUT>(
                    FLIGHT_LANDING_TIMEOUT, TOPIC_FLIGHT);

            Actuators::getInstance().buzzerLanded();

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
            CanHandler::getInstance().sendCamOffCommand();
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
            Actuators::getInstance().buzzerLanded();
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
            CanHandler::getInstance().sendCamOffCommand();
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
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
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

}  // namespace Main
