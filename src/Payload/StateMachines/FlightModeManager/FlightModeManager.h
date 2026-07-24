/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <Payload/PersistentVars/PersistentVars.h>
#include <diagnostic/PrintLogger.h>
#include <events/HSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "FlightModeManagerData.h"

namespace Payload
{
class Sensors;
class CanHandler;
class Actuators;
class AltitudeTrigger;
class FlightStatsRecorder;
class NASController;

/**
 * @brief State machine that manages the flight modes of the Payload.
 */
class FlightModeManager
    : public Boardcore::HSM<FlightModeManager>,
      public Boardcore::InjectableWithDeps<Sensors, CanHandler, Actuators,
                                           AltitudeTrigger, FlightStatsRecorder,
                                           NASController>
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
     * @return Whether manual updates to the reference (altitude, latitude and
     * longitude) are allowed.
     */
    bool referenceUpdateAllowed() const;

    /**
     * @return Whether arbitrary servo movements are allowed.
     */
    bool servoMovesAllowed() const;

    /**
     * @brief Super state for when the Payload is on the ground.
     */
    Boardcore::State OnGround(const Boardcore::Event& event);

    /**
     * @brief The Payload is initializing.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundInit(const Boardcore::Event& event);

    /**
     * @brief Initialization has failed.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundInitError(const Boardcore::Event& event);

    /**
     * @brief Initialization completed successfully.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundInitDone(const Boardcore::Event& event);

    /**
     * @brief Sensors are being calibrated.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundSensorCalibration(const Boardcore::Event& event);

    /**
     * @brief Algorithms are being calibrated.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundAlgorithmCalibration(
        const Boardcore::Event& event);

    /**
     * @brief The Payload is disarmed on the ground, ready to be armed.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundDisarmed(const Boardcore::Event& event);

    /**
     * @brief The Payload is in test mode, allowing execution of test commands
     * that wouldn't be safe to perform in flight mode.
     *
     * Super state: OnGround
     */
    Boardcore::State OnGroundTestMode(const Boardcore::Event& event);

    /**
     * @brief Super state for when the Payload is armed and ready to fly.
     * Algorithms start to run (NAS) and the detach event is awaited to enter
     * the flying state.
     */
    Boardcore::State Armed(const Boardcore::Event& event);

    /**
     * @brief Super state for when the Payload is flying after lift-off.
     */
    Boardcore::State Flying(const Boardcore::Event& event);

    /**
     * @brief Ascending phase of the flight.
     *
     * Super state: Flying
     */
    Boardcore::State FlyingAscending(const Boardcore::Event& event);

    /**
     * @brief Apogee has been reached, expulsion charge has been fired and the
     * drogue parachute was deployed.
     *
     * Super state: Flying
     */
    Boardcore::State FlyingDrogueDescent(const Boardcore::Event& event);

    /**
     * @brief The parafoil wing is deployed by firing cutters and the wing
     * controller is activated.
     *
     * Super state: Flying
     */
    Boardcore::State FlyingWingDescent(const Boardcore::Event& event);

    /**
     * @brief The Payload has landed.
     */
    Boardcore::State Landed(const Boardcore::Event& event);

private:
    void updateState(FlightModeManagerState newState);

    std::atomic<FlightModeManagerState> state{
        FlightModeManagerState::ON_GROUND};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("FMM");
};
}  // namespace Payload
