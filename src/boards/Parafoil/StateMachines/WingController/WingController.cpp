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
#include "WingController.h"

#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WESConfig.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/WingAlgorithm.h>
#include <Parafoil/Wing/WingAlgorithmData.h>
#include <Parafoil/Wing/WingTargetPositionData.h>
#include <common/events/Events.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Parafoil::WingConfig;
using namespace Parafoil::WESConfig;
using namespace Common;
using namespace miosix;

namespace Parafoil
{

WingController::WingController()
    : FSM(&WingController::state_idle), running(false), selectedAlgorithm(0)
{

    EventBroker::getInstance().subscribe(this, TOPIC_ALGOS);
    // setting up the 2 type of algorithm
    addAlgorithm(new AutomaticWingAlgorithm(0.1, 0.01, PARAFOIL_LEFT_SERVO,
                                            PARAFOIL_RIGHT_SERVO));
    WingAlgorithm* timedDescent = new WingAlgorithm(
        PARAFOIL_LEFT_SERVO,
        PARAFOIL_RIGHT_SERVO);  // TODO encapsulate in a method
    WingAlgorithmData step;
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    step.timestamp   = 0;
    timedDescent->addStep(step);
    step.servo1Angle = 120;
    step.servo2Angle = 120;
    step.timestamp += WingConfig::WING_STRAIGHT_FLIGHT_TIMEOUT;
    timedDescent->addStep(step);
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    step.timestamp += WingConfig::WING_STRAIGHT_FLIGHT_TIMEOUT;
    timedDescent->addStep(step);
    addAlgorithm(timedDescent);

    targetPosition[0] = DEFAULT_TARGET_LAT;
    targetPosition[1] = DEFAULT_TARGET_LON;

    // Register the task
    BoardScheduler::getInstance().getScheduler().addTask(
        std::bind(&WingController::update, this), WING_UPDATE_PERIOD);
}

WingController::~WingController()
{
    EventBroker::getInstance().unsubscribe(this);
}

WingControllerState WingController::getStatus()
{
    PauseKernelLock lock;
    return status.state;
}

void WingController::state_idle(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(WingControllerState::IDLE);
        }
        case WING_WES:
        {
            return transition(&WingController::state_wes);
        }
    }
}
void WingController::state_wes(
    const Boardcore::Event& event)  // TODO Redo the FSM and create a HSM with a
                                    // end state called by FMM:state_Landed
{
    switch (event)
    {
        case EV_ENTRY:  // starts twirling and calibration wes
        {
            Actuators::getInstance().startTwirl();
            EventBroker::getInstance().postDelayed<WES_TIMEOUT>(WING_CONTROLLED,
                                                                TOPIC_ALGOS);
            WindEstimation::getInstance()
                .startWindEstimationSchemeCalibration();
            return logStatus(WingControllerState::WES);
        }
        case WING_WES_CALIBRATION:  // stop calibration and start wes
        {

#ifndef PRF_TEST
            WindEstimation::getInstance().stopWindEstimationSchemeCalibration();
            WindEstimation::getInstance().startWindEstimationScheme();
            return logStatus(WingControllerState::WES);
#endif
        }
        case WING_CONTROLLED:  // stop twirling
        {
            Actuators::getInstance().stopTwirl();
            logStatus(WingControllerState::WES);
            if (controlled)
            {
                return transition(&WingController::state_automatic);
            }
            else
            {
                return transition(&WingController::state_file);
            }
        }
    }
}
void WingController::state_automatic(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:  // start automatic algorithm
        {
            selectAlgorithm(0);
            AltitudeTrigger::getInstance().enable();
            startAlgorithm();
            return logStatus(WingControllerState::AUTOMATIC);
        }
        case FLIGHT_WING_ALT_PASSED:  // stop it and return to wes
        {
            stopAlgorithm();
            AltitudeTrigger::getInstance().disable();
            return transition(&WingController::state_wes);
        }  // start the algorithm, inside we add the task to the scheduler
    }
}

void WingController::state_file(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:  // start file algorithm
        {
            selectAlgorithm(1);
            startAlgorithm();
            return logStatus(WingControllerState::FILE);
        }
        case ALGORITHM_ENDED:  // stop it and return to wes
        {
            stopAlgorithm();
            return transition(&WingController::state_wes);
        }
    }
}

void WingController::setControlled(bool controlled)
{
    this->controlled = controlled;
}

void WingController::addAlgorithm(WingAlgorithm* algorithm)
{
    // Ensure that the servos are correct
    algorithm->setServo(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    // Init the algorithm
    algorithm->init();

    // Add the algorithm to the vector
    algorithms.push_back(algorithm);
}

void WingController::selectAlgorithm(size_t index)
{
    stopAlgorithm();
    if (index < algorithms.size())
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

void WingController::startAlgorithm()
{
    // If the selected algorithm is valid --> also the
    // algorithms array is not empty i start the whole thing
    if (selectedAlgorithm < algorithms.size())
    {
        running = true;

        // Begin the selected algorithm
        algorithms[selectedAlgorithm]->begin();

        LOG_INFO(logger, "Wing algorithm started");
    }
}

void WingController::stopAlgorithm()
{
    if (running)
    {
        // Set running to false
        running = false;
        // Stop the algorithm if selected
        if (selectedAlgorithm < algorithms.size())
        {
            algorithms[selectedAlgorithm]->end();
            reset();
        }
    }
}

void WingController::update()
{
    if (running)
    {
        algorithms[selectedAlgorithm]->step();
    }
}

void WingController::flare()
{
    // Set the servo position to flare (pull the two ropes as skydiving people
    // do)
    Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 1);
    Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 1);
}

void WingController::reset()
{
    // Set the servo position to reset
    Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 0);
    Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 0);
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

void WingController::logStatus(WingControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

Eigen::Vector2f WingController::getTargetPosition() { return targetPosition; }

}  // namespace Parafoil
