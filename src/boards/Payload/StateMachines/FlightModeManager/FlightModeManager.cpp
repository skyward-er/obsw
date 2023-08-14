/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Federico Mandelli
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
#include <Payload/Configs/FlightModeManagerConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace miosix;
using namespace Boardcore;
using namespace Common;

namespace Payload
{
FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_on_ground)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_CAN);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
}

FlightModeManager::~FlightModeManager()
{
    EventBroker::getInstance().unsubscribe(this);
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
            return transition(&FlightModeManager::state_init_done);
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
            ModuleManager::getInstance().get<Actuators>()->rocketSDError();
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
            // ModuleManager::getInstance().get<CanHandler>()->sendEvent(
            //     CanConfig::EventId::FORCE_INIT);
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
            // We turn off the led if we are coming from init_error
            ModuleManager::getInstance().get<Actuators>()->rocketSDOff();
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
            ModuleManager::getInstance().get<Sensors>()->calibrate();
            EventBroker::getInstance().post(FMM_ALGOS_CALIBRATE, TOPIC_FMM);
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
        case FMM_ALGOS_CALIBRATE:
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
            // EventBroker::getInstance().post(FMM_ALGOS_CALIBRATE, TOPIC_FMM);
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
            EventBroker::getInstance().post(FMM_READY, TOPIC_FMM);
            return transition(&FlightModeManager::state_disarmed);
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
            // Stop eventual logging
            Logger::getInstance().stop();
            ModuleManager::getInstance().get<Actuators>()->rocketSDDisarmed();
            ModuleManager::getInstance().get<Actuators>()->camOff();
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
        case CAN_ARM:
        case TMTC_ARM:
        {
            // TODO SPAM CAN events when received by radio
            return transition(&FlightModeManager::state_armed);
        }
        case CAN_ENTER_TEST_MODE:
        case TMTC_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::state_test_mode);
        }
        case CAN_CALIBRATE:
        case TMTC_CALIBRATE:
        {
            return transition(&FlightModeManager::state_sensors_calibration);
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
        case TMTC_START_RECORDING:
        {
            ModuleManager::getInstance().get<Actuators>()->camOn();
            return HANDLED;
        }
        case TMTC_STOP_RECORDING:
        {
            ModuleManager::getInstance().get<Actuators>()->camOff();
            return HANDLED;
        }
        case TMTC_RESET_BOARD:
        {
            Logger::getInstance().stop();
            reboot();
            return HANDLED;
        }
        case CAN_EXIT_TEST_MODE:
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
            // Starts signaling devices and camera
            ModuleManager::getInstance().get<Actuators>()->rocketSDArmed();
            ModuleManager::getInstance().get<Actuators>()->camOn();
            // Post event
            EventBroker::getInstance().post(FLIGHT_ARMED, TOPIC_FLIGHT);
            EventBroker::getInstance().post(NAS_FORCE_START, TOPIC_NAS);
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
            EventBroker::getInstance().post(NAS_FORCE_STOP, TOPIC_NAS);
            return transition(&FlightModeManager::state_disarmed);
        }
        case CAN_LIFTOFF:
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
    switch (event)
    {
        case EV_ENTRY:
        {
            EventBroker::getInstance().postDelayed(
                FLIGHT_MISSION_TIMEOUT, TOPIC_FLIGHT, MISSION_TIMEOUT);
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
            return transition(&FlightModeManager::state_ascending);
        }
        case TMTC_FORCE_LANDING:
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

State FlightModeManager::state_ascending(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::ASCENDING);
            EventBroker::getInstance().post(FMM_ASCENDING, TOPIC_FMM);
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
        case CAN_APOGEE_DETECTED:
        case FLIGHT_NC_DETACHED:
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

            EventBroker::getInstance().post(FLIGHT_DROGUE_DESCENT,
                                            TOPIC_FLIGHT);
            EventBroker::getInstance().post(FLIGHT_NC_DETACHED, TOPIC_FLIGHT);
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
        case TMTC_FORCE_DEPLOYMENT:
        {
            return transition(&FlightModeManager::state_wing_descent);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::state_wing_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            EventBroker::getInstance().post(FLIGHT_WING_DESCENT, TOPIC_FLIGHT);

            logStatus(FlightModeManagerState::WING_DESCENT);
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

            // Turns off signaling devices
            ModuleManager::getInstance().get<Actuators>()->rocketSDLanded();
            ModuleManager::getInstance().get<Actuators>()->camOff();
            // Sends events
            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);
            EventBroker::getInstance().postDelayed(FMM_STOP_LOGGING, TOPIC_FMM,
                                                   LOGGING_DELAY);

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
        case FMM_STOP_LOGGING:
        {
            Logger::getInstance().stop();
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

void FlightModeManager::logStatus(FlightModeManagerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

}  // namespace Payload
