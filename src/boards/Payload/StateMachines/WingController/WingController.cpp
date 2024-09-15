/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto
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
#include <Payload/Configs/ActuatorsConfig.h>
#include <Payload/Configs/WingConfig.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/Wing/AutomaticWingAlgorithm.h>
#include <Payload/Wing/FileWingAlgorithm.h>
#include <Payload/Wing/WingAlgorithm.h>
#include <Payload/Wing/WingAlgorithmData.h>
#include <Payload/Wing/WingTargetPositionData.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;
using namespace Payload::Config::Wing;
using namespace Payload::Config::Actuators;

namespace Payload
{

WingController::WingController()
    : HSM(&WingController::Idle, miosix::STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::wingControllerPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    EventBroker::getInstance().subscribe(this, TOPIC_WING);

    // Instantiate the algorithms
    loadAlgorithms();
}

WingController::~WingController()
{
    EventBroker::getInstance().unsubscribe(this);
}

State WingController::Idle(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(WingControllerState::IDLE);
            return HANDLED;
        }

        case FLIGHT_WING_DESCENT:
        {
            return transition(&WingController::Flying);
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

State WingController::Flying(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return HANDLED;
        }

        case EV_EXIT:
        {
            if (cuttersOffEventId != 0)
            {
                EventBroker::getInstance().removeDelayed(cuttersOffEventId);
            }

            getModule<Actuators>()->cuttersOff();

            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&WingController::state_top);
        }

        case EV_INIT:
        {
            return transition(&WingController::FlyingDeployment);
        }

        case DPL_CUT_TIMEOUT:
        {
            getModule<Actuators>()->cuttersOff();
            return HANDLED;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&WingController::OnGround);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}

State WingController::FlyingDeployment(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(WingControllerState::FLYING_DEPLOYMENT);

            getModule<Actuators>()->cuttersOn();
            cuttersOffEventId = EventBroker::getInstance().postDelayed(
                DPL_CUT_TIMEOUT, TOPIC_DPL,
                milliseconds{CUTTERS_TIMEOUT}.count());

            auto nasState  = getModule<NASController>()->getNasState();
            float altitude = -nasState.d;
            getModule<FlightStatsRecorder>()->deploymentDetected(
                TimestampTimer::getTimestamp(), altitude);

            flareEventId = EventBroker::getInstance().postDelayed(
                DPL_FLARE_START, TOPIC_DPL, milliseconds{FLARE_WAIT}.count());

            return HANDLED;
        }

        case EV_EXIT:
        {
            if (flareEventId != 0)
            {
                EventBroker::getInstance().removeDelayed(flareEventId);
            }
            resetWing();

            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&WingController::Flying);
        }

        case DPL_FLARE_START:
        {
            flareWing();
            flareEventId = EventBroker::getInstance().postDelayed(
                DPL_FLARE_STOP, TOPIC_DPL,
                milliseconds{FLARE_DURATION}.count());

            return HANDLED;
        }

        case DPL_FLARE_STOP:
        {
            resetWing();
            flareCount--;

            if (flareCount > 0)
            {
                flareEventId = EventBroker::getInstance().postDelayed(
                    DPL_FLARE_START, TOPIC_DPL,
                    milliseconds{FLARE_INTERVAL}.count());
            }
            else
            {
                EventBroker::getInstance().post(DPL_DONE, TOPIC_DPL);
            }

            return HANDLED;
        }

        case DPL_DONE:
        {
            return transition(&WingController::FlyingControlledDescent);
        }

        default:
        {
            return UNHANDLED;
        }
    }
}
State WingController::FlyingControlledDescent(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(WingControllerState::FLYING_CONTROLLED_DESCENT);

            startAlgorithm();
            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&WingController::Flying);
        }

        case WING_ALGORITHM_ENDED:
        {
            return transition(&WingController::OnGround);
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

State WingController::OnGround(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(WingControllerState::ON_GROUND);

            stopAlgorithm();
            resetWing();

            getModule<Actuators>()->disableServo(PARAFOIL_LEFT_SERVO);
            getModule<Actuators>()->disableServo(PARAFOIL_RIGHT_SERVO);

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

    bool algoStarted =
        std::all_of(algorithms.begin(), algorithms.end(),
                    [](auto& algorithm) { return algorithm->init(); });

    if (!algoStarted)
    {
        LOG_ERR(logger, "Failed to initialize wing algorithms");
        return false;
    }

    auto updateTask = scheduler.addTask([this] { update(); }, UPDATE_RATE);

    if (updateTask == 0)
    {
        LOG_ERR(logger, "Failed to add wing controller update task");
        return false;
    }

    auto activeTargetTask = scheduler.addTask(
        [this]
        {
            // Do not update the active target if the wing is not flying
            if (!running)
            {
                return;
            }

            auto nasState  = getModule<NASController>()->getNasState();
            float altitude = -nasState.d;
            emGuidance.updateActiveTarget(altitude);
        },
        10_hz);

    if (activeTargetTask == 0)
    {
        LOG_ERR(logger, "Failed to add early maneuver active target task");
        return false;
    }

    if (!HSM::start())
    {
        LOG_ERR(logger, "Failed to start WingController HSM active object");
        return false;
    }

    started = true;
    return true;
}

bool WingController::isStarted() { return started; }

WingControllerState WingController::getState() { return state; }

Eigen::Vector2f WingController::getTargetCoordinates()
{
    return targetPositionGEO.load();
}

bool WingController::setTargetCoordinates(float latitude, float longitude)
{
    // Allow changing the target position in the IDLE state only
    if (state != WingControllerState::IDLE)
    {
        return false;
    }

    targetPositionGEO = Coordinates{latitude, longitude};

    // Log early maneuver points to highlight any discrepancies if any
    auto earlyManeuverPoints = getEarlyManeuverPoints();

    auto data = WingTargetPositionData{
        .targetLat = latitude,
        .targetLon = longitude,
        .targetN   = earlyManeuverPoints.targetN,
        .targetE   = earlyManeuverPoints.targetE,
        .emcN      = earlyManeuverPoints.emcN,
        .emcE      = earlyManeuverPoints.emcE,
        .m1N       = earlyManeuverPoints.m1N,
        .m1E       = earlyManeuverPoints.m1E,
        .m2N       = earlyManeuverPoints.m2N,
        .m2E       = earlyManeuverPoints.m2E,
    };
    Logger::getInstance().log(data);

    return true;
}

uint8_t WingController::getSelectedAlgorithm()
{
    return static_cast<uint8_t>(selectedAlgorithm.load());
}

bool WingController::selectAlgorithm(uint8_t index)
{
    // Allow changing the algorithm in the IDLE state only
    if (state != WingControllerState::IDLE)
    {
        return false;
    }

    switch (index)
    {
        case static_cast<uint8_t>(AlgorithmId::EARLY_MANEUVER):
        case static_cast<uint8_t>(AlgorithmId::CLOSED_LOOP):
        case static_cast<uint8_t>(AlgorithmId::ROTATION):
        {
            selectedAlgorithm = static_cast<AlgorithmId>(index);

            auto data = WingControllerAlgorithmData{
                .timestamp = TimestampTimer::getTimestamp(),
                .algorithm = index};
            Logger::getInstance().log(data);

            return true;
        }

        default:
        {
            return false;
        }
    }
}

EarlyManeuversPoints WingController::getEarlyManeuverPoints()
{
    return emGuidance.getPoints();
}

Eigen::Vector2f WingController::getActiveTarget()
{
    return emGuidance.getActiveTarget();
}

void WingController::loadAlgorithms()
{
    // Early Maneuver Guidance Automatic Algorithm
    algorithms[static_cast<size_t>(AlgorithmId::EARLY_MANEUVER)] =
        std::make_unique<AutomaticWingAlgorithm>(
            PI::KP, PI::KI, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO,
            emGuidance);

    // Closed Loop Guidance Automatic Algorithm
    algorithms[static_cast<size_t>(AlgorithmId::CLOSED_LOOP)] =
        std::make_unique<AutomaticWingAlgorithm>(
            PI::KP, PI::KI, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO,
            clGuidance);

    // Rotation Sequence
    {
        auto algorithm = std::make_unique<WingAlgorithm>(PARAFOIL_LEFT_SERVO,
                                                         PARAFOIL_RIGHT_SERVO);
        WingAlgorithmData step;

        step.timestamp   = 0;
        step.servo1Angle = LeftServo::ROTATION / 2;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        step.timestamp += microseconds{ROTATION_PERIOD}.count();
        step.servo1Angle = 0;
        step.servo2Angle = RightServo::ROTATION / 2;
        algorithm->addStep(step);

        step.timestamp += microseconds{ROTATION_PERIOD}.count();
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        step.timestamp += microseconds{ROTATION_PERIOD}.count();
        step.servo1Angle = LeftServo::ROTATION / 2;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        step.timestamp += microseconds{ROTATION_PERIOD}.count();
        step.servo1Angle = 0;
        step.servo2Angle = RightServo::ROTATION / 2;
        algorithm->addStep(step);

        step.timestamp += microseconds{2min}.count();
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        algorithms[static_cast<size_t>(AlgorithmId::ROTATION)] =
            std::move(algorithm);
    }
}

WingAlgorithm& WingController::getCurrentAlgorithm()
{
    auto index = static_cast<size_t>(selectedAlgorithm.load());
    return *algorithms[index].get();
}

void WingController::startAlgorithm()
{
    updateEarlyManeuverPoints();
    running = true;

    getCurrentAlgorithm().begin();
}

void WingController::stopAlgorithm()
{
    if (running)
    {
        running = false;

        getCurrentAlgorithm().end();
    }
}

void WingController::updateEarlyManeuverPoints()
{
    using namespace Eigen;

    auto nas       = getModule<NASController>();
    auto nasState  = nas->getNasState();
    auto nasRef    = nas->getReferenceValues();
    auto targetGEO = targetPositionGEO.load();

    Vector2f currentPositionNED = {nasState.n, nasState.e};
    Vector2f targetNED          = Aeroutils::geodetic2NED(
                 targetGEO, {nasRef.refLatitude, nasRef.refLongitude});

    Vector2f targetOffsetNED = targetNED - currentPositionNED;
    Vector2f normPoint       = targetOffsetNED / targetOffsetNED.norm();
    float psi0               = atan2(normPoint.y(), normPoint.x());

    float distFromCenterline = 20;  // the distance that the M1 and M2 points
                                    // must have from the center line

    // Calculate the angle between the lines <NED Origin, target> and <NED
    // Origin, M1> This angle is the same for M2 since is symmetric to M1
    // relatively to the center line
    float psiMan = atan2(distFromCenterline, targetOffsetNED.norm());

    float maneuverPointsMagnitude = distFromCenterline / sin(psiMan);
    float m2Angle                 = psi0 + psiMan;
    float m1Angle                 = psi0 - psiMan;

    // EMC is calculated as target * 1.2
    Vector2f emcPosition = targetOffsetNED * 1.2 + currentPositionNED;

    Vector2f m1Position =
        Vector2f{cos(m1Angle), sin(m1Angle)} * maneuverPointsMagnitude +
        currentPositionNED;

    Vector2f m2Position =
        Vector2f{cos(m2Angle), sin(m2Angle)} * maneuverPointsMagnitude +
        currentPositionNED;

    emGuidance.setPoints(targetNED, emcPosition, m1Position, m2Position);
    clGuidance.setPoints(targetNED);

    // Log the updated points
    auto data = WingTargetPositionData{
        .targetLat = targetGEO.latitude,
        .targetLon = targetGEO.longitude,
        .targetN   = targetNED.x(),
        .targetE   = targetNED.y(),
        .emcN      = emcPosition.x(),
        .emcE      = emcPosition.y(),
        .m1N       = m1Position.x(),
        .m1E       = m1Position.y(),
        .m2N       = m2Position.x(),
        .m2E       = m2Position.y(),
    };
    Logger::getInstance().log(data);
}

void WingController::update()
{
    if (running)
    {
        getCurrentAlgorithm().step();
    }
}

void WingController::flareWing()
{
    getModule<Actuators>()->setServoPosition(PARAFOIL_LEFT_SERVO, 1.0f);
    getModule<Actuators>()->setServoPosition(PARAFOIL_RIGHT_SERVO, 1.0f);
}

void WingController::resetWing()
{
    getModule<Actuators>()->setServoPosition(PARAFOIL_LEFT_SERVO, 0.0f);
    getModule<Actuators>()->setServoPosition(PARAFOIL_RIGHT_SERVO, 0.0f);
}

void WingController::updateState(WingControllerState newState)
{
    state = newState;

    auto status = WingControllerStatus{
        .timestamp = TimestampTimer::getTimestamp(),
        .state     = newState,
    };
    Logger::getInstance().log(status);
}

}  // namespace Payload
