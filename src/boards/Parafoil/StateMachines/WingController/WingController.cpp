/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli, Radu Raul, Angelo Prete
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
#include <Parafoil/Configs/ActuatorsConfigs.h>
#include <Parafoil/Configs/WESConfig.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/FileWingAlgorithm.h>
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
using namespace Parafoil::ActuatorsConfigs;
using namespace Common;
using namespace miosix;

namespace Parafoil
{

WingController::WingController(TaskScheduler* sched)
    : HSM(&WingController::state_idle), running(false), selectedAlgorithm(0),
      scheduler(sched)
{

    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    EventBroker::getInstance().subscribe(this, TOPIC_WING);
    this->targetPositionGEO = {DEFAULT_TARGET_LAT, DEFAULT_TARGET_LON};
}

bool WingController::start()
{
    return scheduler->addTask(std::bind(&WingController::update, this),
                              WING_UPDATE_PERIOD) &&
           addAlgorithms() && HSM::start();
}

WingController::~WingController()
{
    EventBroker::getInstance().unsubscribe(this);
}

WingControllerStatus WingController::getStatus()
{
    miosix::Lock<miosix::FastMutex> s(statusMutex);
    return status;
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
        case FLIGHT_WING_DESCENT:
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
    static uint16_t calibrationTimeoutEventId;

    switch (event)
    {
        case EV_ENTRY:  // starts twirling and calibration wes
        {
            logStatus(WingControllerState::CALIBRATION);

            flare();
            calibrationTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_SERVO_ACTUATION_DETECTED, TOPIC_DPL, 2000);

            return HANDLED;
        }
        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(calibrationTimeoutEventId);
            return HANDLED;
        }
        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_flying);
        }
        case DPL_SERVO_ACTUATION_DETECTED:
        {
            reset();
            calibrationTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_WIGGLE, TOPIC_DPL, 1000);

            return HANDLED;
        }
        case DPL_WIGGLE:
        {
            flare();
            calibrationTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_NC_OPEN, TOPIC_DPL, 2000);

            return HANDLED;
        }
        case DPL_NC_OPEN:
        {
            reset();
            // calibrationTimeoutEventId =
            // EventBroker::getInstance().postDelayed(
            //    DPL_WES_CAL_DONE, TOPIC_DPL, WES_CALIBRATION_TIMEOUT);
            // modules.get<WindEstimation>()
            //    ->startWindEstimationSchemeCalibration();

            // modules.get<Actuators>()->startTwirl();

            return transition(&WingController::state_controlled_descent);
        }
        case DPL_WES_CAL_DONE:
        {
            modules.get<Actuators>()->stopTwirl();

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
        case WING_ALGORITHM_ENDED:
        {
            return transition(&WingController::state_on_ground);
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
        case EV_ENTRY:
        {
            logStatus(WingControllerState::ON_GROUND);

            ModuleManager::getInstance()
                .get<WindEstimation>()
                ->stopWindEstimationScheme();
            ModuleManager::getInstance()
                .get<WindEstimation>()
                ->stopWindEstimationSchemeCalibration();
            stopAlgorithm();

            // disable servos
            ModuleManager::getInstance().get<Actuators>()->disableServo(
                PARAFOIL_LEFT_SERVO);
            ModuleManager::getInstance().get<Actuators>()->disableServo(
                PARAFOIL_RIGHT_SERVO);

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

bool WingController::addAlgorithms()
{
    WingAlgorithm* algorithm;
    WingAlgorithmData step;

    bool result = false;

    // Algorithm 0
    algorithm = new AutomaticWingAlgorithm(KP, KI, PARAFOIL_LEFT_SERVO,
                                           PARAFOIL_RIGHT_SERVO, clGuidance);
    result    = algorithm->init();
    algorithms.push_back(algorithm);
    // Algorithm 1
    algorithm = new AutomaticWingAlgorithm(KP, KI, PARAFOIL_LEFT_SERVO,
                                           PARAFOIL_RIGHT_SERVO, emGuidance);
    result &= algorithm->init();
    algorithms.push_back(algorithm);

    // Algorithm 2
    algorithm = new WingAlgorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);
    step.servo1Angle = 0;
    step.servo2Angle = 120;
    step.timestamp   = 0;
    algorithm->addStep(step);
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    step.timestamp += 1000 * WingConfig::WING_STRAIGHT_FLIGHT_TIMEOUT;
    algorithm->addStep(step);
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    step.timestamp += WingConfig::WING_STRAIGHT_FLIGHT_TIMEOUT;
    algorithm->addStep(step);
    result &= algorithm->init();
    // Add the algorithm to the vector
    algorithms.push_back(algorithm);

    // Algorithm 3 (rotation)
    algorithm = new WingAlgorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);
    step.timestamp   = 0;
    step.servo1Angle = LEFT_SERVO_ROTATION / 2;
    step.servo2Angle = 0;
    algorithm->addStep(step);
    step.timestamp += WES_ROTATION_PERIOD;  // us
    step.servo1Angle = 0;
    step.servo2Angle = RIGHT_SERVO_ROTATION / 2;
    algorithm->addStep(step);
    step.timestamp += WES_ROTATION_PERIOD;  // us
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    algorithm->addStep(step);
    step.timestamp += WES_ROTATION_PERIOD;  // us
    step.servo1Angle = LEFT_SERVO_ROTATION;
    step.servo2Angle = RIGHT_SERVO_ROTATION;
    algorithm->addStep(step);
    step.timestamp += WES_ROTATION_PERIOD;  // us
    step.servo1Angle = 0;
    step.servo2Angle = RIGHT_SERVO_ROTATION;
    algorithm->addStep(step);
    step.timestamp += WES_ROTATION_PERIOD;  // us
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    algorithm->addStep(step);
    step.timestamp += WES_ROTATION_PERIOD;  // us
    step.servo1Angle = 0;
    step.servo2Angle = 0;
    algorithm->addStep(step);
    result &= algorithm->init();
    algorithms.push_back(algorithm);

    // Algorithm 4 (Progressive rotation)
    algorithm = new WingAlgorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    step.timestamp = 5000 * 1000;  // us

    for (int i = 150; i >= 0; i -= PARAFOIL_WING_DECREMENT)
    {
        step.servo1Angle = i;
        step.servo2Angle = 0;
        algorithm->addStep(step);
        step.timestamp += PARAFOIL_COMMAND_PERIOD * 1000;  // us
        step.servo1Angle = 0;
        step.servo2Angle = i;
        algorithm->addStep(step);
        step.timestamp += PARAFOIL_COMMAND_PERIOD * 1000;  // us
    }
    result &= algorithm->init();
    algorithms.push_back(algorithm);

    selectAlgorithm(SELECTED_ALGORITHM);

    return result;
}

bool WingController::selectAlgorithm(unsigned int index)
{
    bool success = false;
    //  We change the selected algorithm only if we are in IDLE
    if (getStatus().state == WingControllerState::IDLE)
    {
        stopAlgorithm();
        if (index < algorithms.size())
        {
            LOG_INFO(logger, "Algorithm {:1} selected", index);
            selectedAlgorithm = index;
            success           = true;
        }
        else
        {
            // Select the 0 algorithm
            selectedAlgorithm = 0;
            success           = false;
        }
    }
    return success;
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
    if (ModuleManager::getInstance().get<NASController>()->getStatus().state ==
        NASControllerState::ACTIVE)
    {
        this->targetPositionGEO = targetGEO;
        setEarlyManeuverPoints(
            convertTargetPositionToNED(targetPositionGEO),
            {ModuleManager::getInstance().get<NASController>()->getNasState().n,
             ModuleManager::getInstance()
                 .get<NASController>()
                 ->getNasState()
                 .e});
    }
}

void WingController::setEarlyManeuverPoints(Eigen::Vector2f targetNED,
                                            Eigen::Vector2f currentPosNED)
{

    Eigen::Vector2f targetOffsetNED = targetNED - currentPosNED;

    Eigen::Vector2f norm_point = targetOffsetNED / targetOffsetNED.norm();

    float psi0 = atan2(norm_point[1], norm_point[0]);

    float distFromCenterline = 20;  // the distance that the M1 and M2 points
                                    // must have from the center line
                                    // TODO add parameter

    // Calculate the angle between the lines <NED Origin, target> and <NED
    // Origin, M1> This angle is the same for M2 since is symmetric to M1
    // relatively to the center line
    float psiMan = atan2(distFromCenterline, targetOffsetNED.norm());

    float maneuverPointsMagnitude = distFromCenterline / sin(psiMan);
    float m2Angle                 = psi0 + psiMan;
    float m1Angle                 = psi0 - psiMan;

    Eigen::Vector2f emcPosition =
        targetOffsetNED * 1.2 +
        currentPosNED;  // EMC is calculated as target * 1.2

    Eigen::Vector2f m1Position =
        Eigen::Vector2f(cos(m1Angle), sin(m1Angle)) * maneuverPointsMagnitude +
        currentPosNED;

    Eigen::Vector2f m2Position =
        Eigen::Vector2f(cos(m2Angle), sin(m2Angle)) * maneuverPointsMagnitude +
        currentPosNED;

    emGuidance.setPoints(targetNED, emcPosition, m1Position, m2Position);

    clGuidance.setPoints(targetNED);

    WingTargetPositionData data;

    data.receivedLat = targetPositionGEO[0];
    data.receivedLon = targetPositionGEO[1];

    data.targetN = targetNED[0];
    data.targetE = targetNED[1];

    data.emcN = emcPosition[0];
    data.emcE = emcPosition[1];

    data.m1N = m1Position[0];
    data.m1E = m1Position[1];

    data.m2N = m2Position[0];
    data.m2E = m2Position[1];

    // Log the received position
    Logger::getInstance().log(data);
}

void WingController::logStatus(WingControllerState state)
{
    miosix::Lock<miosix::FastMutex> s(statusMutex);
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

Eigen::Vector2f WingController::convertTargetPositionToNED(
    Eigen::Vector2f targetGEO)
{
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
