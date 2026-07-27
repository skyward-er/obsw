/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto, Federico Lolli
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

#include <Main/Configs/WingConfig.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <algorithms/AlgorithmsData.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <prf/PRF.h>
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
 */

namespace Main
{
class BoardScheduler;
class Actuators;
class NASController;
class Sensors;
class LandingFlare;

class WingController
    : public Boardcore::FSM<WingController>,
      public Boardcore::InjectableWithDeps<
          BoardScheduler, Actuators, NASController, StatsRecorder, Sensors>
{
private:
    enum class FlareType
    {
        FULL,
        PUMP
    };

    struct servoCommand
    {
        float leftCommand;
        float rightCommand;
    };

public:
    /**
     * @brief Initializes the wing controller.
     */
    WingController();

    ~WingController();

    // /**
    //  * @brief Override the inject method to inject dependencies into the
    //  * algorithms, which are instantiated later than top-level modules.
    //  */
    // void inject(Boardcore::DependencyInjector& injector) override;

    bool start() override;

    bool isStarted();

    WingControllerState getState();

    servoCommand getLastServoCommands() { return lastServoCommands; }

    /**
     * @brief Returns the target coordinates.
     * @return The GEO coordinates of the active target.
     */
    Eigen::Vector2f getTargetCoordinates();

    /**
     * @brief Sets the target coordinates.
     */
    bool setTargetCoordinates(float latitude, float longitude);

private:
    // HSM states
    void state_init(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_deployment(const Boardcore::Event& event);
    void state_opening_pumps_pull(const Boardcore::Event& event);
    void state_opening_pumps_release(const Boardcore::Event& event);
    void state_guided_descent(const Boardcore::Event& event);
    void state_landing_flare(const Boardcore::Event& event);
    void state_landed(const Boardcore::Event& event);

    /**
     * @brief Periodic update method that steps the currently selected
     * algorithm.
     */
    void update();

    /**
     * @brief Flare the wing.
     * Pulls the two ropes as skydiving people do.
     */
    void flareWing(WingController::FlareType type);

    /**
     * @brief Reset the wing to the initial position.
     */
    void resetWing();

    void updateAndLogStatus(WingControllerState newState);

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

    std::atomic<WingControllerState> state{WingControllerState::INIT};

    uint16_t pumpCount = 0;

    // std::initializer_list<Meter>::const_iterator tinyPullThresholdsIt;

    // uint16_t calibrationTimeoutEventId = 0;
    // uint16_t cuttersOffEventId         = 0;
    uint16_t dplPumpsTimeoutEventId = 0;
    // uint16_t ctrlFlareTimeoutEventId   = 0;
    // uint16_t resetTimeoutEventId       = 0;

    std::atomic<bool> servosStarted{false};

    std::atomic<bool> started{false};

    PRF::PRF wing;

    servoCommand lastServoCommands = {0.0f, 0.0f};

    Boardcore::Logger& sdLogger = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("WingController");
};

}  // namespace Main
