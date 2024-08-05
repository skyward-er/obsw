/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <Main/Actuators/Actuators.h>
#include <Main/AlgoReference/AlgoReference.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/PersistentVars/PersistentVars.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <events/EventBroker.h>
#include <events/HSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "FlightModeManagerData.h"

namespace Main
{

class FlightModeManager
    : public Boardcore::InjectableWithDeps<Actuators, Sensors, CanHandler,
                                           StatsRecorder, AlgoReference,
                                           PersistentVars>,
      public Boardcore::HSM<FlightModeManager>
{
public:
    FlightModeManager();

    FlightModeManagerState getState();

private:
    Boardcore::State state_on_ground(const Boardcore::Event& event);
    Boardcore::State state_init(const Boardcore::Event& event);
    Boardcore::State state_init_error(const Boardcore::Event& event);
    Boardcore::State state_init_done(const Boardcore::Event& event);
    Boardcore::State state_calibrate_sensors(const Boardcore::Event& event);
    Boardcore::State state_calibrate_algorithms(const Boardcore::Event& event);
    Boardcore::State state_disarmed(const Boardcore::Event& event);
    Boardcore::State state_test_mode(const Boardcore::Event& event);
    Boardcore::State state_armed(const Boardcore::Event& event);
    Boardcore::State state_flying(const Boardcore::Event& event);
    Boardcore::State state_powered_ascent(const Boardcore::Event& event);
    Boardcore::State state_unpowered_ascent(const Boardcore::Event& event);
    Boardcore::State state_drogue_descent(const Boardcore::Event& event);
    Boardcore::State state_terminal_descent(const Boardcore::Event& event);
    Boardcore::State state_landed(const Boardcore::Event& event);

    void updateAndLogStatus(FlightModeManagerState state);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("fmm");

    bool nasReady = false;
    bool adaReady = false;

    uint16_t missionTimeoutEvent = 0;
    uint16_t engineShutdownEvent = 0;
    uint16_t apogeeTimeoutEvent  = 0;
    uint16_t cutterTimeoutEvent  = 0;

    std::atomic<FlightModeManagerState> state{
        FlightModeManagerState::ON_GROUND};
};

}  // namespace Main