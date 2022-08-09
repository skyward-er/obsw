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
#pragma once

#include <Payload/FlightModeManager/FlightModeManagerData.h>
#include <Singleton.h>
#include <common/events/Events.h>
#include <common/events/Topics.h>
#include <events/HSM.h>

namespace Payload
{
class FlightModeManager : public Boardcore::HSM<FlightModeManager>,
                          public Boardcore::Singleton<FlightModeManager>
{
    friend class Boardcore::Singleton<FlightModeManager>;

public:
    FlightModeManagerStatus getStatus();

    // Super state when the payload is on ground with the rocket
    Boardcore::State state_on_ground(const Boardcore::Event& event);

    // Initialization state
    Boardcore::State state_init(const Boardcore::Event& event);

    // State in which the init has failed
    Boardcore::State state_init_error(const Boardcore::Event& event);

    // Sensor calibration state, it could be requested via radio
    Boardcore::State state_sensors_calibration(const Boardcore::Event& event);

    // Algorithm calibration state, it could be requested via radio
    Boardcore::State state_algorithms_calibration(
        const Boardcore::Event& event);

    // Ready state waiting for liftoff event
    Boardcore::State state_ready(const Boardcore::Event& event);

    // Test mode state where certain things are enabled
    Boardcore::State state_test_mode(const Boardcore::Event& event);

    // Super state when the payload is flying
    Boardcore::State state_flying(const Boardcore::Event& event);

    // State in which the payload is waiting for the apogee/detach event
    Boardcore::State state_ascending(const Boardcore::Event& event);

    // State in which the wing algorithm is armed but not active yet
    Boardcore::State state_drogue_descent(const Boardcore::Event& event);

    // State in which the wing algorithm is triggered
    Boardcore::State state_wing_descent(const Boardcore::Event& event);

    // Final landing state
    Boardcore::State state_landed(const Boardcore::Event& event);

private:
    // Constructor
    FlightModeManager();

    // Destructor
    ~FlightModeManager();

    // Log the state inside the SD
    void logStatus(FlightModeManagerState state);

    // The actual status
    FlightModeManagerStatus status;

    // Debug logger
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("fmm");
};
}  // namespace Payload
