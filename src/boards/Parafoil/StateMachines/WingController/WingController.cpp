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
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Parafoil::WingConfig;
using namespace Parafoil::WESConfig;
using namespace Common;
using namespace miosix;

namespace Parafoil
{

WingController::WingController()
    : HSM(&WingController::state_idle), running(false), selectedAlgorithm(0)
{
}

bool WingController::start()
{

    ModuleManager& modules = ModuleManager::getInstance();
    EventBroker::getInstance().subscribe(this, TOPIC_ALGOS);
    // setting up the 2 type of algorithm

    targetPosition[0] = DEFAULT_TARGET_LAT;
    targetPosition[1] = DEFAULT_TARGET_LON;

    // Register the task
    BoardScheduler::getInstance().getScheduler().addTask(
        std::bind(&WingController::update, this), WING_UPDATE_PERIOD);

    return true;
}

WingController::~WingController()
{
    EventBroker::getInstance().unsubscribe(this);
}

WingControllerStatus WingController::getStatus()
{
    PauseKernelLock lock;
    return status;  // TODO checks if the lock is needed
}

State WingController::state_idle(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(WingControllerState::IDLE);
            return HANDLED;
        }
        case FLIGHT_WING_ALT_PASSED:
        {
            return transition(&WingController::state_flying);
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_top);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State WingController::state_flying(const Event& event)
{

    switch (event)
    {
        case EV_ENTRY:
        {
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_top);
        }
        case EV_INIT:
        {
            return transition(&WingController::state_calibration);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&WingController::state_on_ground);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State WingController::state_calibration(const Boardcore::Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();

    switch (event)
    {
        case EV_ENTRY:  // starts twirling and calibration wes
        {
            modules.get<Actuators>()->startTwirl();
            EventBroker::getInstance().postDelayed<WES_TIMEOUT>(WING_CONTROLLED,
                                                                TOPIC_ALGOS);
            modules.get<WindEstimation>()
                ->startWindEstimationSchemeCalibration();
            return logStatus(WingControllerState::WES);
        }
        case EV_EXIT:
        {

#ifndef PRF_TEST
            modules.get<WindEstimation>()
                ->stopWindEstimationSchemeCalibration();
            modules.get<WindEstimation>()->startWindEstimationScheme();
            return logStatus(WingControllerState::WES);
#endif
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_flying);
        }
        case WING_WES_CALIBRATION:
        {
            modules.get<Actuators>()->stopTwirl();
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
State WingController::state_controlled_descent(const Boardcore::Event& event)
{
    ModuleManager& modules = ModuleManager::getInstance();

    switch (event)
    {
        case EV_ENTRY:  // start automatic algorithm
        {
            logStatus(WingControllerState::ALGORITHM_CONTROLLED);
            selectAlgorithm(0);
            modules.get<AltitudeTrigger>()->enable();
            startAlgorithm();
            return HANDLED;
        }
        case ALGORITHM_ENDED:
        case FLIGHT_WING_ALT_PASSED:  // stop it and return to wes
        {
            stopAlgorithm();
            modules.get<AltitudeTrigger>()->disable();
            return transition(&WingController::state_wes);
        }  // start the algorithm, inside we add the task to the scheduler
    }
}

State WingController::state_on_ground(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:  // start automatic algorithm
        {
            logStatus(WingControllerState::ON_GROUND);
            WindEstimation::getInstance().stopWindEstimationScheme();
            WindEstimation::getInstance().stopWindEstimationSchemeCalibration();
            stopAlgorithm();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_top);
        }
        default:
        {
            return UNHANDLED;
        }
    }
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
    Actuators& actuators = *(ModuleManager::getInstance().get<Actuators>());

    // Set the servo position to flare (pull the two ropes as skydiving people
    // do)
    actuators.setServo(PARAFOIL_LEFT_SERVO, 1);
    actuators.setServo(PARAFOIL_RIGHT_SERVO, 1);
}

void WingController::reset()
{
    Actuators& actuators = *(ModuleManager::getInstance().get<Actuators>());

    // Set the servo position to reset
    actuators.setServo(PARAFOIL_LEFT_SERVO, 0);
    actuators.setServo(PARAFOIL_RIGHT_SERVO, 0);
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
