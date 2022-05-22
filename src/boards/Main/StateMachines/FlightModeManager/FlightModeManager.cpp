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

#include <Main/events/Events.h>

using namespace Boardcore;

namespace Main
{

State FlightModeManager::state_initialization(const Event& event)
{
    return transition(&FlightModeManager::state_on_ground);
}

State FlightModeManager::state_on_ground(const Event& event)
{
    switch (event)
    {
        case EV_INIT:
        {
            // TODO: Log state
            return transition(&FlightModeManager::state_init);
        }
        case TMTC_RESET_BOARD:
        {
            Logger::getInstance().stop();
            miosix::reboot();
        }
        default:  // If an event is not handled here, try with super-state
        {
            // Since this is an outer super-state, the parent is HSM_top
            return tranSuper(&FlightModeManager::Hsm_top);
        }
    }

    return HANDLED;
}

State state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
            /* code */
            break;

        default:
            break;
    }
}

State state_init_error(const Event& event) {}

State state_sensors_calibration(const Event& event) {}

State state_algos_calibration(const Event& event) {}

State state_disarmed(const Event& event) {}

State state_armed(const Event& event) {}

State state_flying(const Event& event) {}

State state_ascending(const Event& event) {}

State state_drouge_descent(const Event& event) {}

State state_terminal_descent(const Event& event) {}

State state_landed(const Event& event) {}

}  // namespace Main