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

#include <Main/Configs/FlightModeManagerConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/events/Events.h>
#include <Main/events/Topics.h>
#include <drivers/timer/TimestampTimer.h>

using namespace miosix;
using namespace Boardcore;

namespace Main
{

FlightModeManagerStatus FlightModeManager::getStatus()
{
    PauseKernelLock lock;
    return status;
}

State FlightModeManager::state_initialization(const Event& event)
{
    return transition(&FlightModeManager::state_on_ground);
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
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_init);
        }
        case TMTC_RESET_BOARD:
        {
            Logger::getInstance().stop();
            miosix::reboot();
            return HANDLED;
        }
        default:
        {
            return tranSuper(&FlightModeManager::Hsm_top);
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
            return tranSuper(&FlightModeManager::state_on_ground);
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
            return HANDLED;
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
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
        case FMM_SENSORS_CAL_DONE:
        {
            return transition(&FlightModeManager::state_algos_calibration);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
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
            return HANDLED;
        }
        case FMM_ALGOS_CAL_DONE:
        {
            return transition(&FlightModeManager::state_disarmed);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
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
            EventBroker::getInstance().post(FLIGHT_DISARMED, TOPIC_FLIGHT);
            return HANDLED;
        }
        case TMTC_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::state_test_mode);
        }
        case TMTC_CAL_SENSORS:
        {
            return transition(&FlightModeManager::state_sensors_calibration);
        }
        case TMTC_CAL_ALGOS:
        {
            return transition(&FlightModeManager::state_algos_calibration);
        }
        case TMTC_ARM:
        {
            return transition(&FlightModeManager::state_armed);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
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
            return HANDLED;
        }
        case TMTC_EXIT_TEST_MODE:
        {
            return transition(&FlightModeManager::state_disarmed);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
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
            EventBroker::getInstance().post(FLIGHT_ARMED, TOPIC_FLIGHT);
            Logger::getInstance().start();
            return HANDLED;
        }
        case TMTC_DISARM:
        {
            return transition(&FlightModeManager::state_disarmed);
        }
        case FLIGHT_LIFTOFF_DETECTED:
        case TMTC_FORCE_LAUNCH:
        {
            return transition(&FlightModeManager::state_ascending);
        }
        default:
        {
            return tranSuper(&FlightModeManager::Hsm_top);
        }
    }
}

State FlightModeManager::state_flying(const Event& event)
{
    static uint16_t missionTimeoutEventId = -1;

    switch (event)
    {
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_ascending);
        }
        case EV_ENTRY:
        {
            missionTimeoutEventId =
                EventBroker::getInstance().postDelayed<MISSION_TIMEOUT>(
                    FMM_MISSION_TIMEOUT, TOPIC_FMM);
            return HANDLED;
        }

        // This ensures that the force commands are always fulfilled when in
        // this super state
        case TMTC_FORCE_DROGUE:
        {
            EventBroker::getInstance().post(DPL_OPEN, TOPIC_DPL);
            return HANDLED;
        }
        case TMTC_FORCE_MAIN:
        {
            EventBroker::getInstance().post(DPL_CUT_DROGUE, TOPIC_DPL);
            return HANDLED;
        }

        case FMM_MISSION_TIMEOUT:
        {
            return transition(&FlightModeManager::state_landed);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(missionTimeoutEventId);
            return HANDLED;
        }
        default:
        {
            return tranSuper(&FlightModeManager::Hsm_top);
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
        case FLIGHT_APOGEE_DETECTED:
        case TMTC_FORCE_DROGUE:
        {
            return transition(&FlightModeManager::state_drogue_descent);
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().post(ABK_DISABLE, TOPIC_ABK);
            return HANDLED;
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_flying);
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
            EventBroker::getInstance().post(DPL_OPEN, TOPIC_DPL);
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
        case FLIGHT_LANDING_DETECTED:
        case TMTC_FORCE_LANDING:
        {
            return transition(&FlightModeManager::state_landed);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_flying);
        }
    }
}

State FlightModeManager::state_landed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::LANDED);
            Logger::getInstance().stop();
            return HANDLED;
        }
        default:
        {
            return tranSuper(&FlightModeManager::Hsm_top);
        }
    }
}

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_initialization)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
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
