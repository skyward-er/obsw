/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include "WingController.h"

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WingConfig.h>

using namespace Boardcore;
using namespace Parafoil::WingConfig;

namespace Parafoil
{

WingController::WingController()
{
    // Set the current running state
    this->running = false;

    // Set the current selected algorithm to 0
    this->selectedAlgorithm = 0;

    // Initialize the servos, enable them,
    // register the task into the task scheduler
    init();
}

WingController::~WingController() {}

void WingController::addAlgorithm(const char* filename)
{
    // Create the algorithm
    WingAlgorithm* algorithm = new WingAlgorithm(filename);

    // Add the algorithm to the vector and init it
    algorithms.push_back(algorithm);

    // If init fails[Because of inexistent file or stuff] doesn't matter
    // Because the algorithm is empty and so it won't do anything
    algorithms[algorithms.size() - 1]->init();
}

void WingController::addAlgorithm(WingAlgorithm* algorithm)
{
    // Add the algorithm to the vector
    algorithms.push_back(algorithm);
}

void WingController::selectAlgorithm(unsigned int index)
{
    if (index < algorithms.size())
        selectedAlgorithm = index;
    else
        selectedAlgorithm = 0;

    LOG_INFO(logger, "Algorithm {:1} selected", index);
}

void WingController::start()
{
    // If the selected algorithm is valid --> also the algorithms array is not
    // empty i start the whole thing
    if (selectedAlgorithm < algorithms.size())
    {
        // Set the boolean that enables the update method to true
        running = true;

        // Begin the selected algorithm
        algorithms[selectedAlgorithm]->begin();

        LOG_INFO(logger, "Wing algorithm started");
    }
}

void WingController::stop()
{
    // Set running to false so that the update method doesn't act
    running = false;

    // Stop the algorithm if selected
    if (selectedAlgorithm < algorithms.size())
        algorithms[selectedAlgorithm]->end();
}

void WingController::flare()
{
    stop();

    Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 0);
    Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 0);
}

void WingController::reset()
{
    stop();

    Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 0);
    Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 0);
}

void WingController::update()
{
    if (!running)
        return;

    // If the selected algorithm is valid i can update it
    if (selectedAlgorithm < algorithms.size())
        algorithms[selectedAlgorithm]->update();
}

void WingController::init()
{
    BoardScheduler::getInstance().getScheduler().addTask(
        [=]() { update(); }, WING_UPDATE_PERIOD, WING_CONTROLLER_ID);
}

void WingController::setTargetPosition(Eigen::Vector2f target)
{
    targetPosition = target;
}

Eigen::Vector2f WingController::getTargetPosition() { return targetPosition; }

}  // namespace Parafoil
