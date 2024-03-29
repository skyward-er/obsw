/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli
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

#include <Payload/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>
#include <Payload/Wing/WingAlgorithm.h>
#include <events/HSM.h>

#include <Eigen/Core>
#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

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

namespace Payload
{
class WingController : public Boardcore::HSM<WingController>,
                       public Boardcore::Module

{

public:
    Boardcore::State state_idle(const Boardcore::Event& event);
    Boardcore::State state_flying(const Boardcore::Event& event);
    Boardcore::State state_calibration(const Boardcore::Event& event);
    Boardcore::State state_controlled_descent(const Boardcore::Event& event);
    Boardcore::State state_on_ground(const Boardcore::Event& event);

    /**
     * @brief Destroy the Wing Controller object.
     */
    ~WingController();

    /**
     * @brief Method to set the target position.
     */
    void setTargetPosition(Eigen::Vector2f target);

    /**
     * @brief Selects the algorithm if present.
     *
     * @param index The algorithms vector index
     *
     * @return true if the selection was successful
     */
    bool selectAlgorithm(unsigned int index);

    WingControllerStatus getStatus();

    /**
     * @brief Construct a new Wing Controller object
     */
    explicit WingController(Boardcore::TaskScheduler* sched);

    bool start() override;

private:
    /**
     * @brief Method to add the algorithm in the list
     *
     * @param algorithm The algorithm with
     * all already done (e.g. steps already registered)
     */
    bool addAlgorithms();

    /**
     * @brief target position getter
     */
    Eigen::Vector2f convertTargetPositionToNED(Eigen::Vector2f targetGEO);

    /**
     * @brief set points needed by the Guidance
     */
    void setEarlyManeuverPoints(Eigen::Vector2f targetNED,
                                Eigen::Vector2f currentPosNED);

    void logStatus(WingControllerState state);

    WingControllerStatus status;

    miosix::FastMutex statusMutex;

    /**
     * @brief Target position
     */
    Eigen::Vector2f targetPositionGEO;

    /**
     * @brief List of loaded algorithms (from SD or not)
     */
    std::vector<WingAlgorithm*> algorithms;

    /**
     * @brief PrintLogger
     */
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("PayloadTest");

    /**
     * @brief Internal running state
     */
    std::atomic<bool> running;
    /**
     * @brief This attribute is modified by the mavlink radio section.
     * The user using the Ground Station can select the pre-enumerated algorithm
     * to execute
     */
    std::atomic<size_t> selectedAlgorithm;

    /**
     * @brief Instance of the Early Maneuver Guidance Algorithm used by
     * AutomaticWingAlgorithm
     */
    EarlyManeuversGuidanceAlgorithm emGuidance;

    /**
     * @brief  starts the selected algorithm
     */
    void startAlgorithm();

    /**
     * @brief Sets the internal state to stop and
     * stops the selected algorithm
     */
    void stopAlgorithm();

    /**
     * @brief Stops any on going algorithm and flares the wing
     */
    void flare();

    /**
     * @brief Resets the servos in their initial position
     */
    void reset();

    void update();

    Boardcore::TaskScheduler* scheduler = nullptr;
};
}  // namespace Payload
