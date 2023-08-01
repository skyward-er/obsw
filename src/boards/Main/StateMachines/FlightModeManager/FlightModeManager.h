/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Angelo Prete, Matteo Pignataro
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

namespace Main
{

class FlightModeManager : public Boardcore::Module,
                          public Boardcore::HSM<FlightModeManager>
{
public:
    FlightModeManager();

    FlightModeManagerStatus getStatus();

    // Super state for when the rocket is on ground
    Boardcore::State state_on_ground(const Boardcore::Event& event);

    // Initialization state
    Boardcore::State state_init(const Boardcore::Event& event);

    // State in which the init has failed
    Boardcore::State state_init_error(const Boardcore::Event& event);

    // State in which the init is done and a calibration event is thrown
    Boardcore::State state_init_done(const Boardcore::Event& event);

    // Calibration of all the sensors (offsets)
    Boardcore::State state_calibrate_sensors(const Boardcore::Event& event);

    // Calibration of all the algorithms (triad for NAS etc..)
    Boardcore::State state_calibrate_algorithms(const Boardcore::Event& event);

    // State in which the electronics is ready to be armed
    Boardcore::State state_disarmed(const Boardcore::Event& event);

    // State in which some actions (like main deployment, expulsion event etc..)
    // are accepted
    Boardcore::State state_test_mode(const Boardcore::Event& event);

    // State in which the algorithms start to run (NAS) and the electronics is
    // ready to fly
    Boardcore::State state_armed(const Boardcore::Event& event);

    // Super state in which the liftoff has been detected
    Boardcore::State state_flying(const Boardcore::Event& event);

    // State in which the the rocket is ascending with the motor on. The
    // automatic shutdown algorithm runs during this phase
    Boardcore::State state_powered_ascent(const Boardcore::Event& event);

    // State in which the motor has been shut down. The airbrakes algorithm is
    // running during this phase
    Boardcore::State state_unpowered_ascent(const Boardcore::Event& event);

    // State in which the apogee has been detected (triggered by the ADA system)
    Boardcore::State state_drogue_descent(const Boardcore::Event& event);

    // State in which the parachute has been opened by an altitude trigger
    Boardcore::State state_terminal_descent(const Boardcore::Event& event);

    // State in which n minutes of time have passed after the liftoff event
    Boardcore::State state_landed(const Boardcore::Event& event);

private:
    void logStatus(FlightModeManagerState state);

    FlightModeManagerStatus status;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("FlightModeManager");
};

}  // namespace Main