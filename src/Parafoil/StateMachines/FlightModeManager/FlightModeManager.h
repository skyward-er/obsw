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

#pragma once

#include <events/HSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "FlightModeManagerData.h"

namespace Parafoil
{
class Sensors;
class Actuators;

/**
 * State machine that manages the flight modes of the Parafoil.
 *
 * HSM Schema:
 *
 * PreFlight
 * ├── PreFlightInit
 * ├── PreFlightInitError
 * ├── PreFlightInitDone
 * ├── PreFlightSensorCalibration
 * └── PreFlightAlgorithmCalibration
 *
 * Ready
 * └── ReadyTestMode
 *
 * Flying
 * └── FlyingWingDescent
 *
 * Landed
 */
class FlightModeManager
    : public Boardcore::HSM<FlightModeManager>,
      public Boardcore::InjectableWithDeps<Sensors, Actuators>
{
public:
    FlightModeManager();
    ~FlightModeManager();

    /**
     * @brief Returns the current state of the FlightModeManager.
     */
    FlightModeManagerState getState();

    bool isTestMode() const;

    /**
     * @brief Super state for when the parafoil has not yet ready for launch.
     */
    Boardcore::State PreFlight(const Boardcore::Event& event);

    /**
     * @brief State in which the parafoil is initializing.
     *
     * Super state: PreFlight
     */
    Boardcore::State PreFlightInit(const Boardcore::Event& event);

    /**
     * @brief State in which the init has failed
     *
     * Super state: PreFlight
     */
    Boardcore::State PreFlightInitError(const Boardcore::Event& event);

    /**
     * @brief State in which the init is done and a calibration event is
     * thrown
     *
     * Super state: PreFlight
     */
    Boardcore::State PreFlightInitDone(const Boardcore::Event& event);

    /**
     * @brief Calibration of all sensors.
     *
     * Super state: PreFlight
     */
    Boardcore::State PreFlightSensorCalibration(const Boardcore::Event& event);

    /**
     * @brief Calibration of all algorithms.
     *
     * Super state: PreFlight
     */
    Boardcore::State PreFlightAlgorithmCalibration(
        const Boardcore::Event& event);

    /**
     * @brief Super state in which the parafoil is waiting to be dropped.
     */
    Boardcore::State Ready(const Boardcore::Event& event);

    /**
     * @brief The parafoil will accept specific telecommands otherwise
     * considered risky.
     *
     * Super state: Ready
     */
    Boardcore::State ReadyTestMode(const Boardcore::Event& event);

    /**
     * @brief Super state for when the Parafoil is flying.
     */
    Boardcore::State Flying(const Boardcore::Event& event);

    /**
     * @brief State in which the parafoil wing is opened and starts guiding
     * itself
     *
     * Super state: Flying
     */
    Boardcore::State FlyingWingDescent(const Boardcore::Event& event);

    /**
     * @brief The parafoil ended the flight and closes the log.
     */
    Boardcore::State Landed(const Boardcore::Event& event);

private:
    void updateState(FlightModeManagerState newState);

    std::atomic<FlightModeManagerState> state{
        FlightModeManagerState::PRE_FLIGHT};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("FMM");
};

}  // namespace Parafoil
