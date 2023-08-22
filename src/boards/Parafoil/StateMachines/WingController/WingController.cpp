/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli, Radu Raul
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
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/WingAlgorithm.h>
#include <Parafoil/Wing/WingAlgorithmData.h>
#include <Parafoil/Wing/WingTargetPositionData.h>
#include <common/Events.h>
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
    : HSM(&WingController::state_idle), running(false), selectedAlgorithm(0)
{

    EventBroker::getInstance().subscribe(this, TOPIC_ALGOS);
    this->targetPositionGEO = {DEFAULT_TARGET_LAT, DEFAULT_TARGET_LON};
}

bool WingController::startModule()
{
    // Register the task
    start();
    return BoardScheduler::getInstance().getScheduler().addTask(
        std::bind(&WingController::update, this), WING_UPDATE_PERIOD);
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
    switch (event)
    {
        case EV_ENTRY:  // starts twirling and calibration wes
        {
            logStatus(WingControllerState::CALIBRATION);
            ModuleManager::getInstance().get<Actuators>()->startTwirl();
            EventBroker::getInstance().postDelayed<WES_TIMEOUT>(
                WING_WES_CALIBRATION, TOPIC_ALGOS);
            ModuleManager::getInstance()
                .get<WindEstimation>()
                ->startWindEstimationSchemeCalibration();
            return HANDLED;
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_flying);
        }
        case WING_WES_CALIBRATION:
        {
            ModuleManager::getInstance().get<Actuators>()->stopTwirl();
            return transition(&WingController::state_controlled_descent);
        }
        default:
        {
            return UNHANDLED;
        }
    }
}
State WingController::state_controlled_descent(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:  // start automatic algorithm
        {
            logStatus(WingControllerState::ALGORITHM_CONTROLLED);
            selectAlgorithm(0);
            setEarlyManeuverPoints(
                convertTargetPositionToNED(targetPositionGEO),
                {ModuleManager::getInstance()
                     .get<NASController>()
                     ->getNasState()
                     .n,
                 ModuleManager::getInstance()
                     .get<NASController>()
                     ->getNasState()
                     .e});
            startAlgorithm();
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_flying);
        }
        case EV_EXIT:
        {
            return HANDLED;
        }
        default:
        {
            return UNHANDLED;
        }
    }
}

State WingController::state_on_ground(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:  // start automatic algorithm
        {
            logStatus(WingControllerState::ON_GROUND);

            ModuleManager::getInstance()
                .get<WindEstimation>()
                ->stopWindEstimationScheme();
            ModuleManager::getInstance()
                .get<WindEstimation>()
                ->stopWindEstimationSchemeCalibration();
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

void WingController::addAlgorithm(int id)
{
    WingAlgorithm* algorithm;
    WingAlgorithmData step;

    switch (id)
    {
        case 0:
            algorithm = new AutomaticWingAlgorithm(
                0.1f, 1, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO, clGuidance);
            setAutomatic(true);
            break;
        case 1:  // straight-> brake
            algorithm =
                new WingAlgorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);
            step.servo1Angle = 0;
            step.servo2Angle = 0;
            step.timestamp   = 0;
            algorithm->addStep(step);
            step.servo1Angle = 120;
            step.servo2Angle = 120;
            step.timestamp += WingConfig::WING_STRAIGHT_FLIGHT_TIMEOUT;
            algorithm->addStep(step);
            step.servo1Angle = 0;
            step.servo2Angle = 0;
            step.timestamp += WingConfig::WING_STRAIGHT_FLIGHT_TIMEOUT;
            algorithm->addStep(step);
            setAutomatic(false);
            break;
        case 2:  // rotation in a verse opposite of the one of calibration
            algorithm =
                new WingAlgorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);
            step.servo1Angle = 0;
            step.servo2Angle = 120;
            step.timestamp   = 0;
            algorithm->addStep(step);
            step.servo1Angle = 0;
            step.servo2Angle = 120;
            step.timestamp += WES_TIMEOUT * 1000;  // conversion from ms to us
            algorithm->addStep(step);
            setAutomatic(false);
            break;
        case 3:
            algorithm = new AutomaticWingAlgorithm(
                0.1f, 1, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO, emGuidance);
            setAutomatic(true);
            break;
        default:  // automatic target
            algorithm = new AutomaticWingAlgorithm(
                0.1f, 1, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO, clGuidance);
            setAutomatic(true);
            break;
    }
    selectAlgorithm(0);

    // Ensure that the servos are correct
    algorithm->setServo(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    // Init the algorithm
    algorithm->init();

    // Add the algorithm to the vector
    algorithms.push_back(algorithm);
}

void WingController::setAutomatic(bool automatic)
{
    this->automatic = automatic;
}

void WingController::selectAlgorithm(unsigned int index)
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

    ModuleManager::getInstance().get<Actuators>()->setServo(PARAFOIL_LEFT_SERVO,
                                                            1);
    ModuleManager::getInstance().get<Actuators>()->setServo(
        PARAFOIL_RIGHT_SERVO, 1);
}

void WingController::reset()
{
    // Set the servo position to reset
    ModuleManager::getInstance().get<Actuators>()->setServo(PARAFOIL_LEFT_SERVO,
                                                            0);
    ModuleManager::getInstance().get<Actuators>()->setServo(
        PARAFOIL_RIGHT_SERVO, 0);
}

void WingController::setTargetPosition(Eigen::Vector2f targetGEO)
{
    this->targetPositionGEO = targetGEO;
}

void WingController::setEarlyManeuverPoints(Eigen::Vector2f targetNED,
                                            Eigen::Vector2f startingPosNED)
{

    targetNED = targetNED - startingPosNED;

    targetNED = targetNED / targetNED.norm();

    float targetAngle = atan2(targetNED[1], targetNED[0]);

    float distFromCenterline = 20;  // the distance that the M1 and M2 points
                                    // must have from the center line

    // Calculate the angle between the lines <NED Origin, target> and <NED
    // Origin, M1> This angle is the same for M2 since is symmetric to M1
    // relatively to the center line
    float psiMan = atan2(distFromCenterline, targetNED.norm());

    float maneuverPointsMagnitude = distFromCenterline / sin(psiMan);
    float m2Angle                 = targetAngle + psiMan;
    float m1Angle                 = targetAngle - psiMan;

    Eigen::Vector2f emcPosition =
        targetNED * 1.2 + startingPosNED;  // EMC is calculated as target * 1.2

    Eigen::Vector2f m1Position =
        Eigen::Vector2f(cos(m1Angle), sin(m1Angle)) * maneuverPointsMagnitude +
        startingPosNED;

    Eigen::Vector2f m2Position =
        Eigen::Vector2f(cos(m2Angle), sin(m2Angle)) * maneuverPointsMagnitude +
        startingPosNED;

    WingTargetPositionData data;
    data.targetN = targetNED[0];
    data.targetE = targetNED[1];

    data.emcN = emcPosition[0];
    data.emcE = emcPosition[1];

    data.m1N = m1Position[0];
    data.m1E = m1Position[1];

    data.m2N = m2Position[0];
    data.m2E = m2Position[1];

    emGuidance.setPoints(targetNED, emcPosition, m1Position, m2Position);
    clGuidance.setPoints(targetNED);
    // Log the received position
    Logger::getInstance().log(data);
}

void WingController::logStatus(WingControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

Eigen::Vector2f WingController::convertTargetPositionToNED(
    Eigen::Vector2f targetGEO)
{
    // We transform target to NED inside the getter to account for a possible
    // change in reference value of the nas
    return Aeroutils::geodetic2NED(targetGEO, {ModuleManager::getInstance()
                                                   .get<NASController>()
                                                   ->getReferenceValues()
                                                   .refLatitude,
                                               ModuleManager::getInstance()
                                                   .get<NASController>()
                                                   ->getReferenceValues()
                                                   .refLongitude});
}

}  // namespace Parafoil
