/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <Payload/FlightModeManager/FlightModeManager.h>

using namespace Boardcore;
using namespace Common;

namespace Payload
{
FlightModeManagerStatus FlightModeManager::getStatus()
{
    miosix::PauseKernelLock lock;
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
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_init);
        }
        case FLIGHT_LIFTOFF:
        {
            return transition(&FlightModeManager::state_flying);
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
            return transition(&FlightModeManager::state_calibration);
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
        case TMTC_FORCE_INIT:
        {
            return transition(&FlightModeManager::state_calibration);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
    }
}

State FlightModeManager::state_calibration(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::CALIBRATION);
            return HANDLED;
        }
        case FMM_SENSORS_CAL_DONE:
        {
            return transition(&FlightModeManager::state_ready);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
    }
}

State FlightModeManager::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::READY);
            return HANDLED;
        }
        case TMTC_CAL_SENSORS:
        {
            return transition(&FlightModeManager::state_calibration);
        }
        case TMTC_ENTER_TEST_MODE:
        {
            return transition(&FlightModeManager::state_test_mode);
        }
        case FLIGHT_LIFTOFF:
        {
            return transition(&FlightModeManager::state_ascending);
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
            return transition(&FlightModeManager::state_ready);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_on_ground);
        }
    }
}

State FlightModeManager::state_flying(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::FLYING);
            return HANDLED;
        }
        case EV_INIT:
        {
            return transition(&FlightModeManager::state_ascending);
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
        case FLIGHT_NC_DETACHED:
        {
            return transition(&FlightModeManager::state_drogue_descent);
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            return transition(&FlightModeManager::state_drogue_descent);
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
            return HANDLED;
        }
        case FLIGHT_WING_ALT_REACHED:
        {
            return transition(&FlightModeManager::state_wing_descent);
        }
        case FMM_MISSION_TIMEOUT:
        {
            return transition(&FlightModeManager::state_landed);
        }
        default:
        {
            return tranSuper(&FlightModeManager::state_flying);
        }
    }
}

State FlightModeManager::state_wing_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(FlightModeManagerState::WING_DESCENT);
            return HANDLED;
        }
        case FLIGHT_TARGET_REACHED:
        {
            return transition(&FlightModeManager::state_landed);
        }
        case FMM_MISSION_TIMEOUT:
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
            return HANDLED;
        }
        default:
        {
            return tranSuper(&FlightModeManager::Hsm_top);
        }
    }
}
}  // namespace Payload