/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto, Federico Lolli
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
#include <Payload/Sensors/Sensors.h>
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
using namespace Payload::Config::Actuators;
using namespace Payload::Config::Wing;

namespace Payload
{

WingController::WingController()
    : HSM(&WingController::Idle, miosix::STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::wingControllerPriority())
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    EventBroker::getInstance().subscribe(this, TOPIC_WING);
    EventBroker::getInstance().subscribe(this, TOPIC_ALT);

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
            // Turn off cutters in the case of an early exit
            EventBroker::getInstance().removeDelayed(cuttersOffEventId);
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
                milliseconds{Config::Wing::CUTTERS_TIMEOUT}.count());

            auto nasState = getModule<NASController>()->getNasState();
            auto altitude = -nasState.d;  // [m]
            getModule<FlightStatsRecorder>()->deploymentDetected(
                TimestampTimer::getTimestamp(), altitude);

            if (Config::Wing::Deployment::PUMPS.size() >
                0)  // If there is at least one pump specified
                dplFlareTimeoutEventId = EventBroker::getInstance().postDelayed(
                    DPL_FLARE_START, TOPIC_DPL,
                    milliseconds{Config::Wing::Deployment::PUMP_DELAY}.count());
            else
                EventBroker::getInstance().post(DPL_DONE, TOPIC_DPL);

            if (Config::Wing::DynamicTarget::ENABLED)
                initDynamicTarget(
                    Config::Wing::DynamicTarget::LATITUDE_OFFSET,
                    Config::Wing::DynamicTarget::LONGITUDE_OFFSET);

            return HANDLED;
        }

        case EV_EXIT:
        {
            // Stop flares in the case of an early exit
            EventBroker::getInstance().removeDelayed(dplFlareTimeoutEventId);
            EventBroker::getInstance().removeDelayed(resetTimeoutEventId);
            EventBroker::getInstance().removeDelayed(calibrationTimeoutEventId);
            resetWing();

            return HANDLED;
        }

        case EV_EMPTY:
        {
            return tranSuper(&WingController::Flying);
        }

        case DPL_FLARE_START:
        {
            auto pump = Config::Wing::Deployment::PUMPS.at(pumpCount);

            flareWing();
            dplFlareTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_FLARE_STOP, TOPIC_DPL,
                milliseconds{pump.flareTime}.count());

            return HANDLED;
        }

        case DPL_FLARE_STOP:
        {
            auto pump = Config::Wing::Deployment::PUMPS.at(pumpCount);

            resetWing();

            if (++pumpCount >= Config::Wing::Deployment::PUMPS.size())
                EventBroker::getInstance().post(DPL_DONE, TOPIC_DPL);
            else
                dplFlareTimeoutEventId = EventBroker::getInstance().postDelayed(
                    DPL_FLARE_START, TOPIC_DPL,
                    milliseconds{pump.resetTime}.count());

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

            // Enable the landing flare altitude trigger
            if (Config::Wing::LandingFlare::ENABLED)
                getModule<LandingFlare>()->enable();

            return HANDLED;
        }

        case EV_EXIT:
        {
            stopAlgorithm();

            if (Config::Wing::LandingFlare::ENABLED)
            {
                EventBroker::getInstance().removeDelayed(
                    ctrlFlareTimeoutEventId);

                getModule<LandingFlare>()->disable();
            }

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

        case ALTITUDE_TRIGGER_ALTITUDE_REACHED:
        {
            pauseAlgorithm();
            flareWing();

            ctrlFlareTimeoutEventId = EventBroker::getInstance().postDelayed(
                WING_LANDING_FLARE_STOP, TOPIC_FLIGHT,
                milliseconds{Config::Wing::LandingFlare::DURATION}.count());

            return HANDLED;
        }

        case WING_LANDING_FLARE_STOP:
        {
            resetWing();
            resumeAlgorithm();

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
        algorithm->inject(injector);
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

    auto updateTask =
        scheduler.addTask([this] { update(); }, Config::Wing::UPDATE_RATE);

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
                return;

            auto nasState  = getModule<NASController>()->getNasState();
            float altitude = -nasState.d;
            emGuidance.updateActiveTarget(altitude);
        },
        Config::Wing::TARGET_UPDATE_RATE);

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
        return false;

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
        return false;

    Config::Wing::AlgorithmId id =
        static_cast<Config::Wing::AlgorithmId>(index);

    switch (id)
    {
        case Config::Wing::AlgorithmId::EARLY_MANEUVER:
        case Config::Wing::AlgorithmId::CLOSED_LOOP:
        case Config::Wing::AlgorithmId::ROTATION:
        {
            selectedAlgorithm = id;

            auto data = WingControllerAlgorithmData{
                .timestamp = TimestampTimer::getTimestamp(),
                .algorithm = index,
            };
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

void WingController::initDynamicTarget(float latitudeOffset,
                                       float longitudeOffset)
{
    auto gps = getModule<Sensors>()->getUBXGPSLastSample();

    // Convert the offset from meters to degrees
    float earthRadius = 6371000;  // [m]
    float metersPerDegreeLongitude =
        earthRadius * Constants::DEGREES_TO_RADIANS *
        cosf(gps.latitude * Constants::DEGREES_TO_RADIANS);
    float metersPerDegreeLatitude = earthRadius * Constants::DEGREES_TO_RADIANS;

    float newLatitude = gps.latitude + latitudeOffset / metersPerDegreeLatitude;
    float newLongitude =
        gps.longitude + longitudeOffset / metersPerDegreeLongitude;

    setTargetCoordinates(newLatitude, newLongitude);
}

void WingController::loadAlgorithms()
{
    using namespace Config::Wing;

    // Closed Loop Guidance Automatic Algorithm
    algorithms[static_cast<size_t>(AlgorithmId::CLOSED_LOOP)] =
        std::make_unique<AutomaticWingAlgorithm>(
            PI::KP, PI::KI, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO,
            clGuidance);

    // Early Maneuver Guidance Automatic Algorithm
    algorithms[static_cast<size_t>(AlgorithmId::EARLY_MANEUVER)] =
        std::make_unique<AutomaticWingAlgorithm>(
            PI::KP, PI::KI, PARAFOIL_LEFT_SERVO, PARAFOIL_RIGHT_SERVO,
            emGuidance);

    // Sequence
    {
        auto algorithm = std::make_unique<WingAlgorithm>(PARAFOIL_LEFT_SERVO,
                                                         PARAFOIL_RIGHT_SERVO);
        WingAlgorithmData step;

        step.timestamp   = 0;
        step.servo1Angle = 0;
        step.servo2Angle = 120;
        algorithm->addStep(step);

        step.timestamp += microseconds{STRAIGHT_FLIGHT_TIMEOUT}.count();
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        step.timestamp += microseconds{STRAIGHT_FLIGHT_TIMEOUT}.count();
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        algorithms[static_cast<size_t>(AlgorithmId::SEQUENCE)] =
            std::move(algorithm);
    }

    // Rotation
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
        step.servo1Angle = LeftServo::ROTATION;
        step.servo2Angle = RightServo::ROTATION;
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
        step.servo1Angle = 0;
        step.servo2Angle = 0;
        algorithm->addStep(step);

        algorithms[static_cast<size_t>(AlgorithmId::ROTATION)] =
            std::move(algorithm);
    }

    // Progressive rotation
    {
        auto algorithm = std::make_unique<WingAlgorithm>(PARAFOIL_LEFT_SERVO,
                                                         PARAFOIL_RIGHT_SERVO);
        WingAlgorithmData step;

        step.timestamp = microseconds{PROGRESSIVE_ROTATION_TIMEOUT}.count();

        for (auto angle = 80; angle >= 0; angle -= WING_DECREMENT)
        {
            step.servo1Angle = angle;
            step.servo2Angle = 0;
            algorithm->addStep(step);
            step.timestamp += microseconds{COMMAND_PERIOD}.count();

            step.servo1Angle = 0;
            step.servo2Angle = angle;
            algorithm->addStep(step);
            step.timestamp += microseconds{COMMAND_PERIOD}.count();
        }

        algorithms[static_cast<size_t>(AlgorithmId::PROGRESSIVE_ROTATION)] =
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

void WingController::pauseAlgorithm() { running = false; }
void WingController::resumeAlgorithm() { running = true; }

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

    // the distance that the M1 and M2 points must have from the center line
    float distFromCenterline = LATERAL_DISTANCE;

    // Calculate the angle between the lines <NED Origin, target> and <NED
    // Origin, M1> This angle is the same for M2 since is symmetric to M1
    // relatively to the center line
    float psiMan = atan2(distFromCenterline, targetOffsetNED.norm());

    float maneuverPointsMagnitude = distFromCenterline / sin(psiMan);
    float m2Angle                 = psi0 + psiMan;
    float m1Angle                 = psi0 - psiMan;

    // EMC is calculated as target * SCALE_FACTOR
    Vector2f emcPosition = targetOffsetNED * SCALE_FACTOR + currentPositionNED;

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
        getCurrentAlgorithm().step();
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
