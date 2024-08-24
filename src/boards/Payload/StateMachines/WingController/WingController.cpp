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

#include <Payload/Actuators/Actuators.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Configs/WESConfig.h>
#include <Payload/Configs/WingConfig.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <Payload/Wing/AutomaticWingAlgorithm.h>
#include <Payload/Wing/FileWingAlgorithm.h>
#include <Payload/Wing/WingAlgorithm.h>
#include <Payload/Wing/WingAlgorithmData.h>
#include <Payload/Wing/WingTargetPositionData.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace Payload::Config::Wing;
using namespace Payload::WESConfig;
using namespace Common;

namespace Payload
{

WingController::WingController()
    : HSM(&WingController::state_idle), running(false), selectedAlgorithm(0)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    this->targetPositionGEO = {DEFAULT_TARGET_LAT, DEFAULT_TARGET_LON};

    // Instantiate the algorithms
    addAlgorithms();
}

void WingController::inject(DependencyInjector& injector)
{
    for (auto& algorithm : algorithms)
    {
        algorithm->inject(injector);
    }
    Super::inject(injector);
}

bool WingController::start()
{
    auto& scheduler = getModule<BoardScheduler>()->wingController();
    bool success    = true;

    success &= std::all_of(algorithms.begin(), algorithms.end(),
                           [](auto& algorithm) { return algorithm->init(); });

    return success &&
           scheduler.addTask([this] { update(); }, WING_UPDATE_PERIOD) &&
           HSM::start();
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
            float alt = -getModule<NASController>()->getNasState().d;

            getModule<Actuators>()->cuttersOn();
            getModule<FlightStatsRecorder>()->deploymentDetected(
                TimestampTimer::getTimestamp(), alt);

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
            getModule<Actuators>()->setServoPosition(
                ServosList::PARAFOIL_LEFT_SERVO, 1);
            getModule<Actuators>()->setServoPosition(
                ServosList::PARAFOIL_RIGHT_SERVO, 0);
            EventBroker::getInstance().postDelayed(DPL_WES_CAL_DONE, TOPIC_DPL,
                                                   WES_CALIBRATION_TIMEOUT);
            getModule<WindEstimation>()->startWindEstimationSchemeCalibration();
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
        case DPL_WES_CAL_DONE:
        {
            reset();
            // Turn off the cutters
            getModule<Actuators>()->cuttersOff();
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
                {getModule<NASController>()->getNasState().n,
                 getModule<NASController>()->getNasState().e});
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
        case EV_ENTRY:
        {
            logStatus(WingControllerState::ON_GROUND);

            getModule<WindEstimation>()->stopWindEstimationScheme();
            getModule<WindEstimation>()->stopWindEstimationSchemeCalibration();
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

void WingController::addAlgorithms()
{
    WingAlgorithm* algorithm;
    WingAlgorithmData step;

    algorithm = new AutomaticWingAlgorithm(KP, KI, PARAFOIL_LEFT_SERVO,
                                           PARAFOIL_RIGHT_SERVO, emGuidance);
    // Ensure that the servos are correct
    algorithm->setServo(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    // Add the algorithm to the vector
    algorithms.push_back(algorithm);

    algorithm = new WingAlgorithm(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    for (int i = 0; i < 3; i++)
    {
        step.servo1Angle = 120;
        step.servo2Angle = 0;
        step.timestamp   = 0 + i * 40000000;
        algorithm->addStep(step);
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        step.timestamp += 10000000 + i * 40000000;
        algorithm->addStep(step);
        step.servo1Angle = 0;
        step.servo2Angle = 120;
        step.timestamp += 10000000 + i * 40000000;
        algorithm->addStep(step);
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        step.timestamp += 10000000 + i * 40000000;
        algorithm->addStep(step);
    }

    // Ensure that the servos are correct
    algorithm->setServo(PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO);

    // Add the algorithm to the vector
    algorithms.push_back(algorithm);

    selectAlgorithm(0);
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
    getModule<Actuators>()->setServoPosition(PARAFOIL_LEFT_SERVO, 1);
    getModule<Actuators>()->setServoPosition(PARAFOIL_RIGHT_SERVO, 1);
}

void WingController::reset()
{
    // Set the servo position to reset
    getModule<Actuators>()->setServoPosition(PARAFOIL_LEFT_SERVO, 0);
    getModule<Actuators>()->setServoPosition(PARAFOIL_RIGHT_SERVO, 0);
}

void WingController::setTargetPosition(Eigen::Vector2f targetGEO)
{
    if (getModule<NASController>()->getState() != NASControllerState::READY)
    {
        this->targetPositionGEO = targetGEO;
        setEarlyManeuverPoints(convertTargetPositionToNED(targetPositionGEO),
                               {getModule<NASController>()->getNasState().n,
                                getModule<NASController>()->getNasState().e});
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
    return Aeroutils::geodetic2NED(
        targetGEO,
        {getModule<NASController>()->getReferenceValues().refLatitude,
         getModule<NASController>()->getReferenceValues().refLongitude});
}

}  // namespace Payload
