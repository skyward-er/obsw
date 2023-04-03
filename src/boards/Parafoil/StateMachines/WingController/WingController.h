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

#include <ActiveObject.h>
#include <Parafoil/ModuleHelper/ParafoilModule.h>
#include <Parafoil/Wing/WingAlgorithm.h>
#include <events/HSM.h>

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
class WingController : public Boardcore::HSM<WingController>,
                       public Boardcore::Singleton<WingController>

{

public:
    Boardcore::State state_idle(const Boardcore::Event& event);
    Boardcore::State state_flying(const Boardcore::Event& event);
    Boardcore::State state_calibration(const Boardcore::Event& event);
    Boardcore::State state_controlled_descent(const Boardcore::Event& event);
    Boardcore::State state_on_ground(const Boardcore::Event& event);

    /**
     * @brief Construct a new Wing Controller object
     */
    WingController();

    /**
     * @brief Destroy the Wing Controller object.
     */
    ~WingController();

    /**
     * @brief Method to set the target position
     */
    void setTargetPosition(Eigen::Vector2f target);

    /**
     * @brief target position getter
     */
    Eigen::Vector2f getTargetPosition();

    /**
     * @brief Selects the algorithm if present.
     *
     * @param index The algorithms vector index
     */
    void selectAlgorithm(unsigned int index);

    WingControllerStatus getStatus();

    /**
     * @brief Method to add the algorithm in the list
     *
     * @param algorithm The algorithm with
     * all already done (e.g. steps already registered)
     */
    void addAlgorithm(WingAlgorithm* algorithm);

    /**
     * @brief Construct a new Wing Controller object
     */
    WingController();

    bool startModule() override;

private:
    void logStatus(WingControllerState state);

    WingControllerStatus status;
    /**
     * @brief Target position
     */
    Eigen::Vector2f targetPosition;

    /**
     * @brief List of loaded algorithms (from SD or not)
     */
    std::vector<WingAlgorithm*> algorithms;

    /**
     * @brief PrintLogger
     */
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("ParafoilTest");

    /**
     * @brief Internal running state
     */
    std::atomic<bool> running;
    /**
     * @brief This attribute is modified by the mavlink radio section.
     * The user using the Ground Station can select the pre-enumered algorithm
     * to execute
     */
    size_t selectedAlgorithm;

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
};
}  // namespace Parafoil
