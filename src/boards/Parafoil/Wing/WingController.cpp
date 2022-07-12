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
#include <Parafoil/Wing/WingConfig.h>
#include <Parafoil/Wing/WingController.h>

using namespace Boardcore;

namespace Parafoil
{

WingController::WingController(TaskScheduler* scheduler)
{
    // Assign the task scheduler
    this->scheduler = scheduler;

    // Set the current running state
    this->running = false;

    // Set the current selected algorithm to 0
    this->selectedAlgorithm = 0;

    // Initialize the servos, enable them,
    // register the task into the task scheduler
    init();
}

WingController::~WingController()
{
    // Delete the servos
    delete servo1;
    delete servo2;
}

void WingController::addAlgorithm(const char* filename)
{
    // Create the algorithm
    WingAlgorithm* algorithm = new WingAlgorithm(servo1, servo2, filename);

    // Add the algorithm to the vector and init it
    algorithms.push_back(algorithm);
    // If init fails[Beacuse of inexistent file or stuff] doesn't matter
    // Because the algorithm is empty and so it won't do anything
    algorithms[algorithms.size() - 1]->init();
}

void WingController::addAlgorithm(WingAlgorithm* algorithm)
{
    // Ensure that the servos are correct
    algorithm->setServo(servo1, servo2);

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
    servo1->set(WING_SERVO_MAX_DEGREES - WING_SERVO1_RESET_POSITION);
    servo2->set(WING_SERVO_MAX_DEGREES - WING_SERVO2_RESET_POSITION);
}

void WingController::reset()
{
    // I stop any on going algorithm
    stop();

    // Set the servo position to reset
    servo1->set(WING_SERVO1_RESET_POSITION);
    servo2->set(WING_SERVO2_RESET_POSITION);
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
    // Instanciate the servos
    servo1 = new WingServo(WING_SERVO1_TIMER, WING_SERVO1_PWM_CHANNEL,
                           WING_SERVO1_MIN_POSITION, WING_SERVO1_MAX_POSITION,
                           WING_SERVO1_RESET_POSITION);

    servo2 = new WingServo(WING_SERVO2_TIMER, WING_SERVO2_PWM_CHANNEL,
                           WING_SERVO2_MIN_POSITION, WING_SERVO2_MAX_POSITION,
                           WING_SERVO2_RESET_POSITION);

    // Enable servos
    servo1->enable();
    servo2->enable();

    // Register the task
    TaskScheduler::function_t updateFunction([=]() { update(); });

    scheduler->addTask(updateFunction, WING_UPDATE_PERIOD, WING_CONTROLLER_ID);
}

void WingController::setTargetPosition(Eigen::Vector2f target)
{
    this->targetPosition = target;
}

Eigen::Vector2f WingController::getTargetPosition() { return targetPosition; }
}  // namespace Parafoil
