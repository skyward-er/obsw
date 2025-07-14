/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "ADAController.h"

#include <Main/Configs/ADAConfig.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <events/EventBroker.h>
#include <utils/AeroUtils/AeroUtils.h>

using namespace Main;
using namespace Boardcore;
using namespace Common;
using namespace miosix;

// The default Kalman is empty, as calibrate will update it correctly
const ADA::KalmanFilter::KalmanConfig DEFAULT_KALMAN_CONFIG{};

ADA::KalmanFilter::KalmanConfig computeADAKalmanConfig(float refPressure)
{
    ADA::KalmanFilter::MatrixNN F;
    ADA::KalmanFilter::MatrixPN H;
    ADA::KalmanFilter::MatrixNN P;
    ADA::KalmanFilter::MatrixNN Q;
    ADA::KalmanFilter::MatrixPP R;
    ADA::KalmanFilter::MatrixNM G;
    ADA::KalmanFilter::CVectorN x;

    // clang-format off
    F  = ADA::KalmanFilter::MatrixNN({
        {1.0, Config::ADA::UPDATE_RATE_SECONDS, 0.5f * Config::ADA::UPDATE_RATE_SECONDS * Config::ADA::UPDATE_RATE_SECONDS},
        {0.0, 1.0,                              Config::ADA::UPDATE_RATE_SECONDS                                          },
        {0.0, 0.0,                              1.0                                                                       }});

    H = {1.0, 0.0, 0.0};

    P = ADA::KalmanFilter::MatrixNN({
        {1.0, 0.0, 0.0}, 
        {0.0, 1.0, 0.0}, 
        {0.0, 0.0, 1.0}});

    Q << ADA::KalmanFilter::MatrixNN({
        {30.0, 0.0,  0.0}, 
        {0.0,  10.0, 0.0}, 
        {0.0,  0.0,  2.5}});

    R[0] = 4000.0;

    G = ADA::KalmanFilter::MatrixNM::Zero();

    x = ADA::KalmanFilter::CVectorN(refPressure, 0, 0);
    // clang-format on

    return {F, H, Q, R, P, G, x};
}

ADAController::ADAController()
    : FSM{&ADAController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::ADA_PRIORITY},
      ada0{DEFAULT_KALMAN_CONFIG}, ada1{DEFAULT_KALMAN_CONFIG},
      ada2{DEFAULT_KALMAN_CONFIG}
{
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool ADAController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getAdaScheduler();

    uint8_t result =
        scheduler.addTask([this]() { update(); }, Config::ADA::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add ADA update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ADA FSM");
        return false;
    }

    ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();
    ada0.setReferenceValues(ref);
    ada1.setReferenceValues(ref);
    ada2.setReferenceValues(ref);

    return true;
}

ADAState ADAController::getADAState(ADANumber num)
{
    Lock<FastMutex> lock{adaMutex};
    switch (num)
    {
        case ADANumber::ADA0:
        {
            return ada0.getState();
        }

        case ADANumber::ADA1:
        {
            return ada1.getState();
        }

        case ADANumber::ADA2:
        {
            return ada2.getState();
        }
        default:
        {
            return ada0.getState();
        }
    }
}

float ADAController::getDeploymentAltitude()
{
    return Config::ADA::DEPLOYMENT_ALTITUDE_TARGET;
}

ADAControllerState ADAController::getState() { return state; }

void ADAController::update()
{
    ADAControllerState curState = state;

    // Lock ADA for the whole duration of the update
    Lock<FastMutex> lock{adaMutex};

    // First update the kalman
    if (curState == ADAControllerState::ARMED ||
        curState == ADAControllerState::SHADOW_MODE ||
        curState == ADAControllerState::ACTIVE_ASCENT ||
        curState == ADAControllerState::ACTIVE_DROGUE_DESCENT ||
        curState == ADAControllerState::ACTIVE_TERMINAL_DESCENT)
    {
        PressureData baro0 = getModule<Sensors>()->getAtmosPressureLastSample(
            Config::Sensors::Atmos::AtmosSensor::SENSOR_0);
        PressureData baro1 = getModule<Sensors>()->getAtmosPressureLastSample(
            Config::Sensors::Atmos::AtmosSensor::SENSOR_1);
        PressureData baro2 = getModule<Sensors>()->getAtmosPressureLastSample(
            Config::Sensors::Atmos::AtmosSensor::SENSOR_2);

        if (baro0.pressureTimestamp > lastBaro0Timestamp)
        {
            // Barometer is valid, correct with it
            ada0.update(baro0.pressure);
        }
        else
        {
            // Do not perform correction
            ada0.update();
        }

        lastBaro0Timestamp = baro0.pressureTimestamp;

        if (baro1.pressureTimestamp > lastBaro1Timestamp)
        {
            // Barometer is valid, correct with it
            ada1.update(baro1.pressure);
        }
        else
        {
            // Do not perform correction
            ada1.update();
        }

        lastBaro1Timestamp = baro1.pressureTimestamp;

        if (baro2.pressureTimestamp > lastBaro2Timestamp)
        {
            // Barometer is valid, correct with it
            ada2.update(baro2.pressure);
        }
        else
        {
            // Do not perform correction
            ada2.update();
        }

        lastBaro2Timestamp = baro2.pressureTimestamp;
    }

    // Then run detections
    if (curState == ADAControllerState::SHADOW_MODE ||
        curState == ADAControllerState::ACTIVE_ASCENT)
    {
        if (ada0.getState().verticalSpeed <
            Config::ADA::APOGEE_VERTICAL_SPEED_TARGET)
        {
            ada0DetectedApogees++;
        }
        else
        {
            // Apogees must be consecutive in order to be valid
            ada0DetectedApogees = 0;
        }

        if (ada1.getState().verticalSpeed <
            Config::ADA::APOGEE_VERTICAL_SPEED_TARGET)
        {
            ada1DetectedApogees++;
        }
        else
        {
            // Apogees must be consecutive in order to be valid
            ada1DetectedApogees = 0;
        }

        if (ada2.getState().verticalSpeed <
            Config::ADA::APOGEE_VERTICAL_SPEED_TARGET)
        {
            ada2DetectedApogees++;
        }
        else
        {
            // Apogees must be consecutive in order to be valid
            ada2DetectedApogees = 0;
        }

        if (curState == ADAControllerState::ACTIVE_ASCENT)
        {
            // Throw events only in ACTIVE_ASCENT
            // We check if at least two out of the three ADA algorithms have
            // detected more than 5 consecutive apogees
            if ((ada0DetectedApogees > Config::ADA::APOGEE_N_SAMPLES &&
                 ada1DetectedApogees > Config::ADA::APOGEE_N_SAMPLES) ||
                (ada0DetectedApogees > Config::ADA::APOGEE_N_SAMPLES &&
                 ada2DetectedApogees > Config::ADA::APOGEE_N_SAMPLES) ||
                (ada1DetectedApogees > Config::ADA::APOGEE_N_SAMPLES &&
                 ada2DetectedApogees > Config::ADA::APOGEE_N_SAMPLES))
            {
                auto gps = getModule<Sensors>()->getUBXGPSLastSample();

                // Notify stats recorder
                getModule<StatsRecorder>()->apogeeDetected(
                    TimestampTimer::getTimestamp(), gps.latitude, gps.longitude,
                    ada0.getState().aglAltitude, ada2.getState().aglAltitude,
                    ada2.getState().aglAltitude);

                EventBroker::getInstance().post(ADA_APOGEE_DETECTED, TOPIC_ADA);
            }
        }
    }

    if (curState == ADAControllerState::ACTIVE_DROGUE_DESCENT)
    {
        if (ada0.getState().aglAltitude < getDeploymentAltitude())
            ada0DetectedDeployments++;
        else
            ada0DetectedDeployments = 0;

        if (ada1.getState().aglAltitude < getDeploymentAltitude())
            ada1DetectedDeployments++;
        else
            ada1DetectedDeployments = 0;

        if (ada2.getState().aglAltitude < getDeploymentAltitude())
            ada2DetectedDeployments++;
        else
            ada2DetectedDeployments = 0;

        if ((ada0DetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES &&
             ada1DetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES) ||
            (ada0DetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES &&
             ada2DetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES) ||
            (ada1DetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES &&
             ada2DetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES))
        {
            // Notify stats recorder
            getModule<StatsRecorder>()->deploymentDetected(
                TimestampTimer::getTimestamp(), ada0.getState().mslAltitude,
                ada1.getState().mslAltitude, ada2.getState().mslAltitude);

            EventBroker::getInstance().post(ADA_DEPLOY_ALTITUDE_DETECTED,
                                            TOPIC_ADA);
        }
    }

    // Log this sample step
    ADAControllerSampleData data = {TimestampTimer::getTimestamp(),
                                    ada0DetectedApogees,
                                    ada1DetectedApogees,
                                    ada2DetectedApogees,
                                    ada0DetectedDeployments,
                                    ada1DetectedDeployments,
                                    ada2DetectedDeployments,
                                    curState};
    sdLogger.log(data);

    // we use a support struct to differentiate between the three ADAs in the
    // logs
    sdLogger.log(ADA0State(ada0.getState()));
    sdLogger.log(ADA1State(ada1.getState()));
    sdLogger.log(ADA2State(ada2.getState()));
}

void ADAController::calibrate()
{
    ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();

    Lock<FastMutex> lock{adaMutex};
    ada0.setReferenceValues(ref);
    ada1.setReferenceValues(ref);
    ada2.setReferenceValues(ref);

    // TODO: Should this be calculated by ADA at the moment?
    auto kalmanConfig = computeADAKalmanConfig(ref.refPressure);
    ada0.setKalmanConfig(kalmanConfig);
    ada1.setKalmanConfig(kalmanConfig);
    ada2.setKalmanConfig(kalmanConfig);

    ada0.update(ref.refPressure);
    ada1.update(ref.refPressure);
    ada2.update(ref.refPressure);

    EventBroker::getInstance().post(ADA_READY, TOPIC_ADA);
}

void ADAController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::INIT);
            break;
        }

        case ADA_CALIBRATE:
        {
            transition(&ADAController::state_calibrating);
            break;
        }
    }
}

void ADAController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::CALIBRATING);
            calibrate();
            break;
        }

        case ADA_READY:
        {
            transition(&ADAController::state_ready);
            break;
        }
    }
}

void ADAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::READY);
            break;
        }

        case ADA_CALIBRATE:
        {
            transition(&ADAController::state_calibrating);
            break;
        }

        case ADA_FORCE_START:
        case FLIGHT_ARMED:
        {
            transition(&ADAController::state_armed);
            break;
        }
    }
}

void ADAController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::ARMED);
            break;
        }

        case ADA_FORCE_STOP:
        {
            transition(&ADAController::state_ready);
            break;
        }

        case FLIGHT_DISARMED:
        {
            transition(&ADAController::state_ready);
            break;
        }

        case FLIGHT_LIFTOFF:
        {
            transition(&ADAController::state_shadow_mode);
            break;
        }
    }
}

void ADAController::state_shadow_mode(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::SHADOW_MODE);

            shadowModeTimeoutEvent = EventBroker::getInstance().postDelayed(
                ADA_SHADOW_MODE_TIMEOUT, TOPIC_ADA,
                Config::ADA::SHADOW_MODE_TIMEOUT);
            break;
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEvent);
            break;
        }

        case ADA_FORCE_STOP:
        {
            transition(&ADAController::state_ready);
            break;
        }

        case ADA_SHADOW_MODE_TIMEOUT:
        {
            transition(&ADAController::state_active_ascent);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ADAController::state_end);
            break;
        }
    }
}

void ADAController::state_active_ascent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::ACTIVE_ASCENT);
            break;
        }

        case ADA_FORCE_STOP:
        {
            transition(&ADAController::state_ready);
            break;
        }

        case FLIGHT_APOGEE_DETECTED:
        {
            transition(&ADAController::state_active_drogue_descent);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ADAController::state_end);
            break;
        }
    }
}

void ADAController::state_active_drogue_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::ACTIVE_DROGUE_DESCENT);
            break;
        }

        case ADA_FORCE_STOP:
        {
            transition(&ADAController::state_ready);
            break;
        }

        case FLIGHT_DPL_ALT_DETECTED:
        {
            transition(&ADAController::state_active_terminal_descent);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ADAController::state_end);
            break;
        }
    }
}

void ADAController::state_active_terminal_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::ACTIVE_TERMINAL_DESCENT);
            break;
        }

        case ADA_FORCE_STOP:
        {
            transition(&ADAController::state_ready);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&ADAController::state_end);
            break;
        }
    }
}

void ADAController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ADAControllerState::END);
            break;
        }
    }
}

void ADAController::updateAndLogStatus(ADAControllerState state)
{
    this->state              = state;
    ADAControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
