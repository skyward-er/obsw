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

#include <Payload/Configs/WingConfig.h>
#include <Payload/Wing/WingController.h>
#include <Payload/Wing/WingTargetPositionData.h>

using namespace Boardcore;
using namespace Payload::WingConfig;

namespace Payload
{

WingController::WingController()
{
    // Assign the task scheduler
    this->scheduler = new TaskScheduler();

    // Set the current running state
    this->running = false;

    // Set the current selected algorithm to 0
    this->selectedAlgorithm = 0;

    // Initialize the servos, enable them,
    // register the task into the task scheduler
    init();
}

WingController::~WingController() {}

void WingController::addAlgorithm(WingAlgorithm* algorithm)
{
    // Ensure that the servos are correct
    algorithm->setServo(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    // Init the algorithm
    algorithm->init();

    // Add the algorithm to the vector
    algorithms.push_back(algorithm);
}

void WingController::selectAlgorithm(unsigned int index)
{
    if (index >= 0 && index < algorithms.size())
    {
        LOG_INFO(logger, "Algorithm {:1} selected", index);
        selectedAlgorithm = index;
    }
    else
    {
        // I select the 0 algorithm
        selectedAlgorithm = 0;
    }
}

void WingController::start()
{
    // If the selected algorithm is valid --> also the
    // algorithms array is not empty i start the whole thing
    if (selectedAlgorithm >= 0 && selectedAlgorithm < algorithms.size())
    {
        // Set the boolean that enables the update method to true
        running = true;

        // Begin the selected algorithm
        algorithms[selectedAlgorithm]->begin();

        // In case i start also the task scheduler
        scheduler->start();

        LOG_INFO(logger, "Wing algorithm started");
    }
}

void WingController::stop()
{
    // Set running to false so that the update method doesn't act
    running = false;
    // Stop the algorithm if selected
    if (selectedAlgorithm >= 0 && selectedAlgorithm < algorithms.size())
    {
        algorithms[selectedAlgorithm]->end();
    }
}

void WingController::flare()
{
    // I stop any on going algorithm
    stop();

    // Set the servo position to flare (pull the two ropes as skydiving people
    // do)
    Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 1);
    Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 1);
}

void WingController::reset()
{
    // I stop any on going algorithm
    stop();

    // Set the servo position to reset
    Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 0);
    Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 0);
}

void WingController::update()
{
    // Check if the algorithm is running
    if (!running)
    {
        return;
    }

    // If the selected algorithm is valid i can update it
    if (selectedAlgorithm >= 0 && selectedAlgorithm < algorithms.size())
    {
        algorithms[selectedAlgorithm]->update();
    }
}

void WingController::init()
{
    // Register the task
    TaskScheduler::function_t updateFunction([=]() { update(); });

    scheduler->addTask(updateFunction, WING_UPDATE_PERIOD, WING_CONTROLLER_ID);

    // Set the target position to the default one
    targetPosition[0] = DEFAULT_TARGET_LAT;
    targetPosition[1] = DEFAULT_TARGET_LON;
}

void WingController::setTargetPosition(Eigen::Vector2f target)
{
    this->targetPosition = target;

    WingTargetPositionData data;
    data.latitude  = target[0];
    data.longitude = target[1];

    // Log the received position
    Logger::getInstance().log(data);
}

Eigen::Vector2f WingController::getTargetPosition() { return targetPosition; }

}  // namespace Payload
