/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Angelo Prete
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

#pragma once

#include "FlightModeManager.h"

#include <Main/Configs/FlightModeManagerConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>

#include "FlightModeManagerData.h"

using namespace Boardcore;
using namespace Common;
using namespace miosix;

namespace Main
{

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_on_ground)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_MOTOR);
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
        //  EV_TC_RESET_BOARD in the design document
        case TMTC_RESET_BOARD:
        {
            // reset()
            // TODO

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
        // EV_INIT_ERROR
        case FMM_INIT_ERROR:
        {
            transition(&FlightModeManager::state_init_error);
        }
        // EV_INIT_OK
        case FMM_INIT_OK:
        {
            transition(&FlightModeManager::state_init_done);
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
        // EV_TC_FORCE_INIT and EV_CAN_FORCE_INIT
        case TMTC_FORCE_INIT:
        {
            transition(&FlightModeManager::state_init_done);
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

            // post(EV_CALIBRATE)
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
        // EV_CALIBRATE
        case FMM_CALIBRATE:
        {
            transition(&FlightModeManager::state_calibrate_sensors);
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

            // calibrateSensors()
            ModuleManager::getInstance().get<Sensors>()->calibrate();

            // post(EV_CALIBRATE_ALGORITHMS)
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
        // EV_CALIBRATE_ALGORITHMS
        case FMM_SENSORS_CAL_DONE:
        {
            transition(&FlightModeManager::state_calibrate_algorithms);
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
            logStatus(FlightModeManagerState::CALIBRATE_ALGORITHMS);

            // calibrateAlgorithms()
            ModuleManager::getInstance().get<NASController>()->calibrate();
            // TODO: Calibrate ADA

            // post(EV_READY)
            EventBroker::getInstance().post(FMM_ALGOS_CAL_DONE, TOPIC_FMM);

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
        // EV_READY
        case FMM_ALGOS_CAL_DONE:
        {
            transition(&FlightModeManager::state_disarmed);
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

            // post(EV_DISARMED)
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
        // EV_TC_CALIBRATE and EV_CAN_CALIBRATE
        case TMTC_CALIBRATE:
        {
            transition(&FlightModeManager::state_calibrate_sensors);
        }
        // EV_TC_ENTER_TEST_MODE and EV_CAN_ENTER_TEST_MODE
        case TMTC_ENTER_TEST_MODE:
        {
            transition(&FlightModeManager::state_test_mode);
        }
        // EV_TC_ARM and EV_CAN_ARM
        case TMTC_ARM:
        {
            transition(&FlightModeManager::state_armed);
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
            logStatus(FlightModeManagerState::INIT_DONE);

            // post(EV_TEST_MODE)
            EventBroker::getInstance().post(, );

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
        // EV_TC_EXIT_TEST_MODE and EV_CAN_EXIT_TEST_MODE
        case TMTC_EXIT_TEST_MODE:
        {
            transition(&FlightModeManager::state_disarmed);
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

            // post(EV_ARMED)
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
        // EV_TC_DISARM and EV_CAN_DISARM
        case TMTC_DISARM:
        {
            transition(&FlightModeManager::state_disarmed);
        }
        // EV_CAN_IGNITION and EV_TC_FORCE_IGNITION
        case:
        {
            transition(&FlightModeManager::state_ignition);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_ignition(const Event& event)
{
    static uint16_t openOxidantTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::IGNITION);

            // post(EV_IGNITION)
            EventBroker::getInstance().post(MOTOR_IGNITION, TOPIC_MOTOR);

            // postD(EV_OPEN_OXIDANT)
            EventBroker::getInstance().postDelayed(
                MOTOR_OPEN_OXIDANT, TOPIC_MOTOR, OPEN_OXIDANT_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(openOxidantTimeoutEventId);

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
        // EV_LIFTOFF
        case FLIGHT_LIFTOFF:
        // EV_TC_FORCE_LAUNCH
        case TMTC_FORCE_LAUNCH:
        {
            transition(&FlightModeManager::state_flying);
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
            logStatus(FlightModeManagerState::FLYING);

            // postD(EV_MISSION_END)
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
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_powered_ascent(const Event& event)
{
    static uint16_t engineShutdownTimeoutEventId = -1;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::POWERED_ASCENT);

            // post(EV_ASCENDING)
            EventBroker::getInstance().post(, );

            // postD(EV_ENGINE_SHUTDOWN)
            engineShutdownTimeoutEventId =
                EventBroker::getInstance().postDelayed(, ,
                                                       ENGINE_SHUTDOWN_TIMEOUT);

            return HANDLED;
        }
        case EV_EXIT:
        {
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
        // EV_TC_FORCE_ENGINE_SHUTDOWN
        case:
        // EV_ENGINE_SHUTDOWN
        case:
        {
            transition(&FlightModeManager::state_unpowered_ascent);
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
            logStatus(FlightModeManagerState::UNPOWERED_ASCENT);

            // post(EV_ENGINE_OFF)
            EventBroker::getInstance().post(, );

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
        // EV_NC_DETACHED
        case FLIGHT_NC_DETACHED:
        // EV_TC_FORCE_APOGEE
        case TMTC_FORCE_APOGEE:
        {
            transition(&FlightModeManager::state_drogue_descent);
        }
        // EV_APOGEE_DETECTED
        case FLIGHT_APOGEE_DETECTED:
        {
            // expulsion
            // TODO

            transition(&FlightModeManager::state_drogue_descent);
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

            // post(EV_DROGUE_DESCENT)
            EventBroker::getInstance().post(, );

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
        // EV_APOGEE_DETECTED
        case FLIGHT_APOGEE_DETECTED:
        {
            // expulsion
            // TODO

            transition(&FlightModeManager::state_terminal_descent);
        }
        // EV_TC_FORCE_DPL
        case:
        // EV_DPL_ALTITUDE_REACHED
        case FLIGHT_DPL_ALT_DETECTED:
        {
            transition(&FlightModeManager::state_terminal_descent);
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

            // post(EV_TERMINAL_DESCENT)
            EventBroker::getInstance().post(, );

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
        // EV_MISSION_END
        case FLIGHT_MISSION_TIMEOUT:
        // EV_TC_FORCE_LANDING
        case TMTC_FORCE_LANDING:
        {
            transition(&FlightModeManager::state_landed);
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
            logStatus(FlightModeManagerState::LANDED);

            // post(EV_LANDED)
            EventBroker::getInstance().post(, );

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
        default:
        {
            return UNHANDLED;
        }
    }
}

void FlightModeManager::logStatus(FlightModeManagerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

}  // namespace Main