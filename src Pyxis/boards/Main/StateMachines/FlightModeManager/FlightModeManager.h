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

#pragma once

#include <Singleton.h>
#include <diagnostic/PrintLogger.h>
#include <events/HSM.h>

#include "FlightModeManagerData.h"

namespace Main
{

class FlightModeManager : public Boardcore::HSM<FlightModeManager>,
                          public Boardcore::Singleton<FlightModeManager>
{
    friend Boardcore::Singleton<FlightModeManager>;

public:
    FlightModeManagerStatus getStatus();

    /// Super state for when the rocket is on ground.
    Boardcore::State state_on_ground(const Boardcore::Event& event);

    /// Initialization state.
    Boardcore::State state_init(const Boardcore::Event& event);

    /// State in which the init has failed.
    Boardcore::State state_init_error(const Boardcore::Event& event);

    /// Calibration of all sensors.
    Boardcore::State state_sensors_calibration(const Boardcore::Event& event);

    /// Calibration of all algorithms.
    Boardcore::State state_algos_calibration(const Boardcore::Event& event);

    /// The rocket is ready for the ARM command.
    Boardcore::State state_disarmed(const Boardcore::Event& event);

    /// The rocket will accept specific telecommands otherwise considered risky.
    Boardcore::State state_test_mode(const Boardcore::Event& event);

    /// The rocket is ready for the liftoff.
    Boardcore::State state_armed(const Boardcore::Event& event);

    /// Super state for when the rocket is in the air.
    Boardcore::State state_flying(const Boardcore::Event& event);

    /// Ascending phase of the trajectory.
    Boardcore::State state_ascending(const Boardcore::Event& event);

    /// Between drogue and main deployment.
    Boardcore::State state_drogue_descent(const Boardcore::Event& event);

    /// After main parachute deployment.
    Boardcore::State state_terminal_descent(const Boardcore::Event& event);

    /// The rocket is on the ground after the flight.
    Boardcore::State state_landed(const Boardcore::Event& event);

    /// The rocket ended the flight and closes the log.
    Boardcore::State state_mission_ended(const Boardcore::Event& event);

private:
    FlightModeManager();
    ~FlightModeManager();

    void logStatus(FlightModeManagerState state);

    FlightModeManagerStatus status;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("fmm");
};

}  // namespace Main
