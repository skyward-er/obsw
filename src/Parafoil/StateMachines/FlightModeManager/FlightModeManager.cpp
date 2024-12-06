/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/FlightModeManagerConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Common;
using namespace std::chrono;
namespace config = Parafoil::Config::FlightModeManager;

namespace Parafoil
{

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::PreFlight, miosix::STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::flightModeManagerPriority())
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

FlightModeManagerState FlightModeManager::getState() { return state; }

bool FlightModeManager::isTestMode() const
{
    return state == FlightModeManagerState::READY_TEST_MODE;
}

State FlightModeManager::PreFlight(const Event& event)
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
            return transition(&FlightModeManager::PreFlightInit);
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

State FlightModeManager::PreFlightInit(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::PRE_FLIGHT_INIT);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::PreFlight);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_INIT_OK:
        {
            return transition(&FlightModeManager::PreFlightInitDone);
        }

        case FMM_INIT_ERROR:
        {
            return transition(&FlightModeManager::PreFlightInitError);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::PreFlightInitError(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::PRE_FLIGHT_INIT_ERROR);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::PreFlight);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_FORCE_INIT:
        {
            return transition(&FlightModeManager::PreFlightInitDone);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::PreFlightInitDone(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::PRE_FLIGHT_INIT_DONE);
            EventBroker::getInstance().post(FMM_CALIBRATE, TOPIC_FMM);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::PreFlight);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_CALIBRATE:
        {
            return transition(&FlightModeManager::PreFlightSensorCalibration);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::PreFlightSensorCalibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(
                FlightModeManagerState::PRE_FLIGHT_ALGORITHM_CALIBRATION);

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
            return tranSuper(&FlightModeManager::PreFlight);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case FMM_ALGOS_CALIBRATE:
        {
            return transition(
                &FlightModeManager::PreFlightAlgorithmCalibration);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::PreFlightAlgorithmCalibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(
                FlightModeManagerState::PRE_FLIGHT_ALGORITHM_CALIBRATION);
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
            return tranSuper(&FlightModeManager::PreFlight);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case NAS_READY:
        {
            EventBroker::getInstance().post(FMM_READY, TOPIC_FMM);
            return transition(&FlightModeManager::Ready);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::Ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::READY);
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

        case TMTC_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::ReadyTestMode);
        }

        case TMTC_CALIBRATE:
        {
            return transition(&FlightModeManager::PreFlightSensorCalibration);
        }

        case FLIGHT_NC_DETACHED:
        case TMTC_FORCE_EXPULSION:
        {
            return transition(&FlightModeManager::FlyingWingDescent);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State FlightModeManager::ReadyTestMode(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::READY_TEST_MODE);
            return HANDLED;
        }

        case EV_EXIT:
        {
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&FlightModeManager::Ready);
        }

        case EV_INIT:
        {
            return HANDLED;
        }

        case TMTC_FORCE_LANDING:
        {
            return transition(&FlightModeManager::Landed);
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
            return transition(&FlightModeManager::FlyingWingDescent);
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

State FlightModeManager::FlyingWingDescent(const Event& event)
{
    static uint16_t controlDelayId;

    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(FlightModeManagerState::FLYING_WING_DESCENT);
            // Send the event to the WingController
            controlDelayId = EventBroker::getInstance().postDelayed(
                FLIGHT_WING_DESCENT, TOPIC_FLIGHT,
                milliseconds{config::CONTROL_DELAY}.count());
            return HANDLED;
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(controlDelayId);
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

        case TMTC_FORCE_LANDING:
        {
            return transition(&FlightModeManager::Landed);
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

            EventBroker::getInstance().post(FLIGHT_LANDING_DETECTED,
                                            TOPIC_FLIGHT);
            EventBroker::getInstance().postDelayed(
                FMM_STOP_LOGGING, TOPIC_FMM,
                milliseconds{config::LOGGING_DELAY}.count());

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

}  // namespace Parafoil