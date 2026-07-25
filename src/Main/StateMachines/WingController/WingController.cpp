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
// namespace LandingFlareConfig = Main::Config::Wing::LandingFlare;
using namespace Boardcore::Units::Length;

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
    // EventBroker::getInstance().subscribe(this, TOPIC_ALT);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);

    // tinyPullThresholdsIt =
    //     LandingFlareConfig::TinyPull::ALTITUDE_THRESHOLDS.begin();
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

    if (!FSM::start())
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
    // Allow changing the target position in the READY state only
    if (state != WingControllerState::READY)
        return false;

    targetPositionGEO = Coordinates{latitude, longitude};

    // getModule<LandingFlare>()->setTargetGEO({latitude, longitude});
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

        // updated servo positions
        // TODO: check if the servos are the correct ones
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
    }
}

void WingController::resetWing()
{
    getModule<Actuators>()->setPrfServoAngle(PARAFOIL_LEFT_SERVO, 0.0_rad);
    getModule<Actuators>()->setPrfServoAngle(PARAFOIL_RIGHT_SERVO, 0.0_rad);
}

void WingController::flareWing(WingController::FlareType type)
{
    switch (type)
    {
        // case FlareType::FULL:
        // {
        //     getModule<Actuators>()->setPrfServoAngle(
        //         PARAFOIL_LEFT_SERVO, LandingFlareConfig::ANGLE_LEFT_SERVO);
        //     getModule<Actuators>()->setPrfServoAngle(
        //         PARAFOIL_RIGHT_SERVO, LandingFlareConfig::ANGLE_RIGHT_SERVO);

        //     return;
        // }
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

void WingController::state_init(const Boardcore::Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(WingControllerState::INIT);

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
            // Coordinates targetReading = targetPositionGEO.load();
            // getModule<LandingFlare>()->setTargetGEO(
            // {targetReading.latitude, targetReading.longitude});
            updateAndLogStatus(WingControllerState::READY);
            break;
        }

        case FMM_ALGOS_CALIBRATE:
        {
            wing.initialize();
            wing.setPRF_Reference({0.0f, Config::Wing::Default::TARGET_LAT,
                                   Config::Wing::Default::TARGET_LON});
            resetWing();

            servosStarted = true;
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

            dplPumpsTimeoutEventId = EventBroker::getInstance().postDelayed(
                DPL_PUMPS_PULL, TOPIC_DPL,
                milliseconds{Config::Wing::Deployment::PUMP_DELAY}.count());

            break;
        }

        case EV_EXIT:
        {
            // Stop pumps in the case of an early exit
            EventBroker::getInstance().removeDelayed(dplPumpsTimeoutEventId);
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

            // // Enable the landing flare altitude trigger
            // if (LandingFlareConfig::ENABLED)
            // {
            //     if (LandingFlareConfig::TinyPull::ENABLED)
            //     {
            //         tinyPullThresholdsIt =
            //             LandingFlareConfig::TinyPull::ALTITUDE_THRESHOLDS
            //                 .begin();
            //         getModule<LandingFlare>()->setDeploymentAltitude(
            //             *tinyPullThresholdsIt);
            //     }
            //     getModule<LandingFlare>()->enable();
            // }

            break;
        }

        case EV_EXIT:
        {
            // if (LandingFlareConfig::ENABLED)
            // {
            //     EventBroker::getInstance().removeDelayed(
            //         ctrlFlareTimeoutEventId);

            //     getModule<LandingFlare>()->disable();
            // }

            break;
        }

        case ALTITUDE_TRIGGER_ALTITUDE_REACHED:
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
            break;
        }

        case EV_EMPTY:
        {
            transition(&WingController::state_ready);
            break;
        }

        case WING_LANDING_FLARE_STOP:
        {
            // resetWing();
            // waitForServosToStop();

            // if (LandingFlareConfig::TinyPull::ENABLED)
            // {
            //     tinyPullThresholdsIt++;
            //     if (tinyPullThresholdsIt ==
            //         LandingFlareConfig::TinyPull::ALTITUDE_THRESHOLDS.end())
            //     {
            //         return HANDLED;
            //     }

            //     getModule<LandingFlare>()->setDeploymentAltitude(
            //         *tinyPullThresholdsIt);
            //     getModule<LandingFlare>()->enable();
            // }

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

