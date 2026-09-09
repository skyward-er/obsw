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

#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/Configs/ActuatorsConfig.h>
#include <Main/Configs/WingConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <units/Length.h>

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;
using namespace Main::Config::Actuators;
using namespace Main::Config::Wing;
using namespace Boardcore::Units::Length;
// using namespace Main::Config::Wing::LandingFlareConfig;

namespace Main
{

WingController::WingController()
    : FSM(&WingController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::NAS_PRIORITY)
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
    EventBroker::getInstance().subscribe(this, TOPIC_DPL);
    EventBroker::getInstance().subscribe(this, TOPIC_WING);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
}

WingController::~WingController() = default;

bool WingController::start()
{
    auto& scheduler = getModule<BoardScheduler>()->getWingScheduler();

    auto updateTask =
        scheduler.addTask([this] { update(); }, Config::Wing::UPDATE_RATE);

    if (updateTask == 0)
    {
        LOG_ERR(logger, "Failed to add wing controller update task");
        return false;
    }

    if (LandingFlareConfig::ENABLED && !altitudeMap.init())
    {
        enableFlare = false;
        LOG_ERR(logger, "Failed to initialize altitude map");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start WingController HSM active object");
        return false;
    }

    return true;
}

WingControllerState WingController::getState() { return state; }

Eigen::Vector2f WingController::getTargetCoordinates()
{
    return targetPositionGEO.load();
}

bool WingController::setTargetCoordinates(float latitude, float longitude)
{
    // Allow changing the target position in the READY state only
    if (state != WingControllerState::READY)
        return false;

    wing.setPRF_Reference({0.0f, latitude, longitude});
    targetPositionGEO = Coordinates{latitude, longitude};
    return true;
}

void WingController::update()
{
    if (state == WingControllerState::GUIDED_DESCENT)
    {
        auto nasdaqState = getModule<NASController>()->getNASDAQState();

        PRFIn input = {
            .NASDAQPosition = {nasdaqState.n, nasdaqState.e, nasdaqState.d},
            .NASDAQVelocity = {nasdaqState.vn, nasdaqState.ve, nasdaqState.vd},
        };

        wing.setPRF_In(input);

        wing.step();

        // retrieve data
        WingControllerLogsData logsData{
            TimestampTimer::getTimestamp(),
            wing.getPRF_Logs_OBSW(),
        };

        lastServoCommands = {logsData.PRFLogs.ServoCommands[0],
                             logsData.PRFLogs.ServoCommands[1]};

        // update servo positions
        Radian leftCommand(logsData.PRFLogs.ServoCommands[0] *
                           Config::Actuators::PrfServo::MAX_ANGLE);
        Radian rightCommand(logsData.PRFLogs.ServoCommands[1] *
                            Config::Actuators::PrfServo::MAX_ANGLE);

        getModule<Actuators>()->setPrfServoAngle(PARAFOIL_LEFT_SERVO,
                                                 leftCommand);
        getModule<Actuators>()->setPrfServoAngle(PARAFOIL_RIGHT_SERVO,
                                                 rightCommand);
        // Log data
        sdLogger.log(logsData);

        // Check if we need to flare

        if (enableFlare && state == WingControllerState::GUIDED_DESCENT)
        {
            auto north = Meter(wing.getTarget_Rel_Position()[0]);
            auto east  = Meter(wing.getTarget_Rel_Position()[1]);

            // Only flare if inside the map boundaries
            if (altitudeMap.isInsideMap(north, east))
            {
                auto aglAltitude =
                    -nasdaqState.d -
                    altitudeMap.getClosestGroundAltitude(north, east)
                        .value();  // [m]

                if (aglAltitude <= LandingFlareConfig::ALTITUDE)
                    flareDetectionCount++;
                else
                    flareDetectionCount = 0;

                if (flareDetectionCount >= LandingFlareConfig::CONFIDENCE)
                {
                    EventBroker::getInstance().post(WING_FLARE_START,
                                                    TOPIC_WING);
                }

                FlareData flareData{TimestampTimer::getTimestamp(), aglAltitude,
                                    flareDetectionCount};
                sdLogger.log(flareData);
            }
        }
    }
}

void WingController::flareWing(WingController::FlareType type)
{
    switch (type)
    {
        case FlareType::FULL:
        {
            getModule<Actuators>()->setPrfServoAngle(
                PARAFOIL_LEFT_SERVO, LandingFlareConfig::FLARE_ANGLE_LEFT);
            getModule<Actuators>()->setPrfServoAngle(
                PARAFOIL_RIGHT_SERVO, LandingFlareConfig::FLARE_ANGLE_RIGHT);

            return;
        }
        case FlareType::PUMP:
        {
            getModule<Actuators>()->setPrfServoAngle(
                PARAFOIL_LEFT_SERVO, Deployment::PUMP_ANGLE_LEFT);
            getModule<Actuators>()->setPrfServoAngle(
                PARAFOIL_RIGHT_SERVO, Deployment::PUMP_ANGLE_RIGHT);
            return;
        }
    }
}

void WingController::resetWing()
{
    getModule<Actuators>()->setPrfServoAngle(PARAFOIL_LEFT_SERVO, 0.0_rad);
    getModule<Actuators>()->setPrfServoAngle(PARAFOIL_RIGHT_SERVO, 0.0_rad);
}

void WingController::state_init(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::INIT);

            wing.initialize();
            wing.setPRF_Reference({0.0f, Config::Wing::Default::TARGET_LAT,
                                   Config::Wing::Default::TARGET_LON});
            resetWing();

            transition(&WingController::state_ready);
            break;
        }
    }
}

void WingController::state_ready(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::READY);
            break;
        }

        case FLIGHT_DPL_ALT_DETECTED:
        {
            transition(&WingController::state_deployment);
            break;
        }
    }
}

void WingController::state_deployment(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::DEPLOYMENT);

            auto altitude =
                -getModule<NASController>()->getNASDAQState().d;  // [m]

            getModule<Actuators>()->enablePrfServo(
                ServosList::PARAFOIL_LEFT_SERVO);
            getModule<Actuators>()->enablePrfServo(
                ServosList::PARAFOIL_RIGHT_SERVO);

            getModule<StatsRecorder>()->deploymentDetected(
                TimestampTimer::getTimestamp(), altitude);

            dplPumpsPullEventId = EventBroker::getInstance().postDelayed(
                DPL_PUMPS_PULL, TOPIC_DPL,
                milliseconds{Config::Wing::Deployment::PUMP_DELAY}.count());

            dplPumpsTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_DONE, TOPIC_DPL,
                milliseconds{Config::Wing::Deployment::PUMP_TIMEOUT}.count());

            break;
        }

        case EV_EXIT:
        {
            // Stop pumps in the case of an early exit
            EventBroker::getInstance().removeDelayed(dplPumpsPullEventId);

            resetWing();

            break;
        }

        case DPL_PUMPS_PULL:
        {
            transition(&WingController::state_opening_pumps_pull);
            break;
        }

        case DPL_DONE:
        {
            transition(&WingController::state_guided_descent);
            break;
        }
    }
}

void WingController::state_opening_pumps_pull(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::OPENING_PUMPS_PULL);

            if (Config::Wing::Deployment::PUMPS.size() <= 0)
            {
                EventBroker::getInstance().post(DPL_DONE, TOPIC_DPL);
                break;
            }

            flareWing(FlareType::PUMP);

            break;
        }

        case PRF_SERVO_STOPPED:
        {
            if (getModule<Actuators>()->arePrfServosStill())
            {
                auto pump = Config::Wing::Deployment::PUMPS.at(pumpCount);

                dplPumpsTimeoutEventId = EventBroker::getInstance().postDelayed(
                    DPL_PUMPS_RELEASE, TOPIC_DPL,
                    milliseconds{pump.pumpTime}.count());
            }

            break;
        }

        case DPL_DONE:
        {
            transition(&WingController::state_guided_descent);
            break;
        }

        case DPL_PUMPS_RELEASE:
        {
            transition(&WingController::state_opening_pumps_release);
            break;
        }
    }
}

void WingController::state_opening_pumps_release(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::OPENING_PUMPS_RELEASE);

            resetWing();

            break;
        }

        case EV_EXIT:
        {
            // Stop pumps in the case of an early exit
            EventBroker::getInstance().removeDelayed(dplPumpsTimeoutEventId);
            resetWing();

            break;
        }

        case PRF_SERVO_STOPPED:
        {
            if (getModule<Actuators>()->arePrfServosStill())
            {
                auto pump = Config::Wing::Deployment::PUMPS.at(pumpCount);

                if (++pumpCount >= Config::Wing::Deployment::PUMPS.size())
                    EventBroker::getInstance().postDelayed(
                        DPL_DONE, TOPIC_DPL,
                        milliseconds{pump.resetTime}.count());
                else
                {
                    dplPumpsTimeoutEventId =
                        EventBroker::getInstance().postDelayed(
                            DPL_PUMPS_PULL, TOPIC_DPL,
                            milliseconds{pump.resetTime}.count());
                }
            }

            break;
        }

        case DPL_DONE:
        {
            transition(&WingController::state_guided_descent);
            break;
        }

        case DPL_PUMPS_PULL:
        {
            transition(&WingController::state_opening_pumps_pull);
            break;
        }
    }
}

void WingController::state_guided_descent(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::GUIDED_DESCENT);
            EventBroker::getInstance().removeDelayed(dplPumpsTimeoutEventId);
            break;
        }

        case EV_EXIT:
        {
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&WingController::state_landed);
            break;
        }

        case WING_FLARE_START:
        {
            transition(&WingController::state_landing_flare);
            break;
        }
    }
}

void WingController::state_landing_flare(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::LANDING_FLARE);

            flareWing(FlareType::FULL);

            EventBroker::getInstance().postDelayed(
                WING_LANDING_FLARE_STOP, TOPIC_WING,
                milliseconds{LandingFlareConfig::DURATION}.count());
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&WingController::state_landed);
            break;
        }

        case WING_LANDING_FLARE_STOP:
        {
            resetWing();

            transition(&WingController::state_landed);
            break;
        }
    }
}

void WingController::state_landed(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::LANDED);

            getModule<Actuators>()->disablePrfServo(PARAFOIL_LEFT_SERVO);
            getModule<Actuators>()->disablePrfServo(PARAFOIL_RIGHT_SERVO);
            break;
        }
    }
}

void WingController::updateAndLogStatus(WingControllerState newState)
{
    state = newState;

    auto status = WingControllerStatus{
        .timestamp = TimestampTimer::getTimestamp(), .state = newState};
    sdLogger.log(status);
}

}  // namespace Main
