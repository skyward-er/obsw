/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto, Davide Basso
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

#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Wing/Guidance/ClosedLoopGuidanceAlgorithm.h>
#include <Parafoil/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>
#include <Parafoil/Wing/WingAlgorithm.h>
#include <diagnostic/PrintLogger.h>
#include <events/HSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <Eigen/Core>
#include <atomic>

#include "WingControllerData.h"

/**
 * @brief This class allows the user to select the wing algorithm
 * that has to be used during the tests. It also registers his
 * dedicated function in the task scheduler in order to be
 * executed every fixed period and to update the two servos position
 * depending on the selected algorithm.
 *
 * Use case example:
 * controller = new WingController(scheduler);
 *
 * controller.addAlgorithm("filename");
 * OR
 * controller.addAlgorithm(algorithm);
 *
 * controller.selectAlgorithm(index);
 *
 * controller.start();
 * controller.stop();  //If you want to abort the execution
 * controller.start(); //If you want to start again from the beginning
 */

namespace Parafoil
{
class BoardScheduler;
class Actuators;
class NASController;
class FlightStatsRecorder;
class WindEstimation;

/**
 * State machine that manages the wings of the Parafoil.
 *
 * HSM Schema:
 *
 * Idle
 *
 * Flying
 * ├── FlyingCalibration
 * ├── FlyingDeployment
 * └── FlyingControlledDescent
 *
 * OnGround
 */
class WingController
    : public Boardcore::HSM<WingController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Actuators,
                                           NASController, FlightStatsRecorder,
                                           WindEstimation>
{
public:
    /**
     * @brief Initializes the wing controller.
     *
     * Sets the initial FSM state to idle, subscribes to DPL and flight events
     * and instantiates the wing algorithms.
     */
    WingController();

    /**
     * @brief Destroy the Wing Controller object.
     */
    ~WingController();

    // HSM states
    Boardcore::State Idle(const Boardcore::Event& event);
    Boardcore::State Flying(const Boardcore::Event& event);
    Boardcore::State FlyingCalibration(const Boardcore::Event& event);
    Boardcore::State FlyingDeployment(const Boardcore::Event& event);
    Boardcore::State FlyingControlledDescent(const Boardcore::Event& event);
    Boardcore::State OnGround(const Boardcore::Event& event);

    /**
     * @brief Override the inject method to inject dependencies into the
     * algorithms, which are instantiated later than top-level modules.
     */
    void inject(Boardcore::DependencyInjector& injector) override;

    bool start() override;

    bool isStarted();

    WingControllerState getState();

    /**
     * @brief Returns the target coordinates.
     * @return The GEO coordinates of the active target.
     */
    Eigen::Vector2f getTargetCoordinates();

    /**
     * @brief Sets the target coordinates.
     */
    bool setTargetCoordinates(float latitude, float longitude);

    /**
     * @brief Returns the selected algorithm.
     */
    uint8_t getSelectedAlgorithm();

    /**
     * @brief Changes the selected algorithm.
     * @return Whether the provided index selected a valid algorithm.
     */
    bool selectAlgorithm(Config::Wing::AlgorithmId id);

    /**
     * @brief Returns the currently set early maneuver points.
     */
    EarlyManeuversPoints getEarlyManeuverPoints();

    /**
     * @brief Returns the early maneuver active target.
     */
    Eigen::Vector2f getActiveTarget();

    /**
     * @brief This is a forward method to the early maneuvers guidance algorithm
     * to calculate the yaw angle of the parafoil knowing the current position
     * and the target position.
     *
     * @param[in] currentPositionNED the current NED position of the parafoil in
     * m
     * @param[out] heading Saves the heading vector for logging purposes
     *
     * @returns the yaw angle of the parafoil in rad
     */
    Boardcore::Units::Angle::Radian calculateTargetAngle(
        const Eigen::Vector3f& currentPositionNED, Eigen::Vector2f& heading)
    {
        return emGuidance.calculateTargetAngle(currentPositionNED, heading);
    }

private:
    /**
     * @brief Loads all algorithms.
     *
     * @note Algorithms should be instantiated before dependency injection so
     * that they can be injected with dependencies when the WingController is
     * injected.
     */
    void loadAlgorithms();

    /**
     * @brief Returns the currently selected algorithm.
     */
    WingAlgorithm& getCurrentAlgorithm();

    /**
     * @brief Starts the currently selected algorithm. If the algorithm is
     * already running, it resets the algorithm.
     */
    void startAlgorithm();

    /**
     * @brief Stops the currently selected algorithm.
     */
    void stopAlgorithm();

    /**
     * @brief Update early maneuver guidance points (EMC, M1, M2) based on the
     * current position and the target position.
     */
    void updateEarlyManeuverPoints();

    /**
     * @brief Periodic update method that steps the currently selected
     * algorithm.
     */
    void update();

    /**
     * @brief Flare the wing.
     * Pulls the two ropes as skydiving people do.
     */
    void flareWing();

    /**
     * @brief Twirl the wing to the left.
     */
    void twirlWing();

    /**
     * @brief Reset the wing to the initial position.
     */
    void resetWing();

    void updateState(WingControllerState newState);

    struct Coordinates
    {
        float latitude;
        float longitude;

        operator Eigen::Vector2f() const { return {latitude, longitude}; }
    };

    std::atomic<Coordinates> targetPositionGEO{Coordinates{
        .latitude  = Config::Wing::Default::TARGET_LAT,
        .longitude = Config::Wing::Default::TARGET_LON,
    }};

    std::atomic<Config::Wing::AlgorithmId> selectedAlgorithm{
        Config::Wing::Default::ALGORITHM};

    std::atomic<WingControllerState> state{WingControllerState::IDLE};

    std::array<std::unique_ptr<WingAlgorithm>,
               static_cast<size_t>(Config::Wing::AlgorithmId::LAST)>
        algorithms;  ///< The available algorithms

    /**
     * @brief Instance of the Early Maneuver Guidance Algorithm used by
     * AutomaticWingAlgorithm
     */
    EarlyManeuversGuidanceAlgorithm emGuidance;

    /**
     * @brief Instance of the Closed Loop Guidance Algorithm used by
     * AutomaticWingAlgorithm
     */
    ClosedLoopGuidanceAlgorithm clGuidance;

    std::atomic<bool> started{false};
    std::atomic<bool> running{false};  ///< Whether the algorithm is running

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("WingController");
};

}  // namespace Parafoil
