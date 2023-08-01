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

#include <events/EventBroker.h>
#include <events/HSM.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "FlightModeManagerData.h"

using namespace Boardcore;

namespace Main
{

class FlightModeManager : public Module, public HSM<FlightModeManager>
{
public:
    FlightModeManager();

    FlightModeManagerStatus getStatus();

    State state_on_ground(const Event& event);
    State state_init(const Event& event);
    State state_init_error(const Event& event);
    State state_init_done(const Event& event);
    State state_calibrate_sensors(const Event& event);
    State state_calibrate_algorithms(const Event& event);
    State state_disarmed(const Event& event);
    State state_test_mode(const Event& event);
    State state_armed(const Event& event);
    State state_ignition(const Event& event);
    State state_flying(const Event& event);
    State state_powered_ascent(const Event& event);
    State state_unpowered_ascent(const Event& event);
    State state_drogue_descent(const Event& event);
    State state_terminal_descent(const Event& event);
    State state_landed(const Event& event);

private:
    void logStatus(FlightModeManagerState state);

    FlightModeManagerStatus status;

    PrintLogger logger = Logging::getLogger("FlightModeManager");
};

}  // namespace Main