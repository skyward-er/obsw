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

#include <algorithm>
#include <chrono>

using namespace std::chrono;
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
      deploymentAltitude{Config::ADA::DEPLOYMENT_ALTITUDE_TARGET},
      shadowModeTime{Config::ADA::SHADOW_MODE_TIMEOUT},
      ada{DEFAULT_KALMAN_CONFIG}
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

    auto algoRef        = getModule<AlgoReference>();
    ReferenceValues ref = algoRef->getReferenceValues();
    ada.setReferenceValues(ref);

    algoRef->subscribeReferenceChanges(this);

    return true;
}

ADAState ADAController::getADAState()
{
    Lock<FastMutex> lock{adaMutex};
    return ada.getState();
}

float ADAController::getDeploymentAltitude()
{
    return deploymentAltitude.load();
}

void ADAController::setDeploymentAltitude(float altitude)
{
    deploymentAltitude = altitude;
}

milliseconds ADAController::getShadowModeTime()
{
    return shadowModeTime.load();
}

void ADAController::setShadowModeTime(milliseconds time)
{
    shadowModeTime = time;
}

ADAControllerState ADAController::getState() { return state; }

void ADAController::onReferenceChanged(const Boardcore::ReferenceValues& ref)
{
    miosix::Lock<miosix::FastMutex> l(adaMutex);
    ada.setReferenceValues(ref);
}

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
        PressureData baro = getModule<Sensors>()->getAtmosPressureLastSample();

        if (baro.pressureTimestamp > lastBaroTimestamp)
        {
            // Barometer is valid, correct with it
            ada.update(baro.pressure);
        }
        else
        {
            // Do not perform correction
            ada.update();
        }

        lastBaroTimestamp = baro.pressureTimestamp;
    }

    auto adaState = ada.getState();

    // Then run detections
    if (curState == ADAControllerState::SHADOW_MODE ||
        curState == ADAControllerState::ACTIVE_ASCENT)
    {
        // Apogees must be consecutive in order to be valid
#define UPDATE_APOGEE_CONFIDENCE(ada)                                         \
    if (ada##State.verticalSpeed < Config::ADA::APOGEE_VERTICAL_SPEED_TARGET) \
        ada##DetectedApogees++;                                               \
    else                                                                      \
        ada##DetectedApogees = 0

        UPDATE_APOGEE_CONFIDENCE(ada);

        if (curState == ADAControllerState::ACTIVE_ASCENT)
        {
            // Throw events only in ACTIVE_ASCENT
            // Detections are "sticky": once an ADA detects an apogee it will
            // be considered as having detected it for the whole flight
            if (adaDetectedApogees > Config::ADA::APOGEE_N_SAMPLES)
            {
                auto gps = getModule<Sensors>()->getUBXGPSLastSample();

                // Notify stats recorder
                getModule<StatsRecorder>()->apogeeDetected(
                    TimestampTimer::getTimestamp(), gps.latitude, gps.longitude,
                    adaState.aglAltitude);

                EventBroker::getInstance().post(ADA_APOGEE_DETECTED, TOPIC_ADA);
            }
        }
    }

    if (curState == ADAControllerState::ACTIVE_DROGUE_DESCENT)
    {
#define UPDATE_DEPLOYMENT_CONFIDENCE(ada)                 \
    if (ada##State.aglAltitude < getDeploymentAltitude()) \
        ada##DetectedDeployments++;                       \
    else                                                  \
        ada##DetectedDeployments = 0

        UPDATE_DEPLOYMENT_CONFIDENCE(ada);

        if (adaDetectedDeployments > Config::ADA::DEPLOYMENT_N_SAMPLES)
        {
            // Notify stats recorder
            getModule<StatsRecorder>()->deploymentDetected(
                TimestampTimer::getTimestamp(), adaState.mslAltitude);

            EventBroker::getInstance().post(ADA_DEPLOY_ALTITUDE_DETECTED,
                                            TOPIC_ADA);
        }
    }

    // Log this sample step
    ADAControllerSampleData data = {TimestampTimer::getTimestamp(),
                                    adaDetectedApogees, adaDetectedDeployments,
                                    curState};
    sdLogger.log(data);

    // Logs ADA states as separate structs
    sdLogger.log(ADAState(adaState));
}

void ADAController::calibrate()
{
    ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();

    Lock<FastMutex> lock{adaMutex};
    ada.setReferenceValues(ref);

    // Recomputing Kalman config also resets the ADA state
    auto kalmanConfig = computeADAKalmanConfig(ref.refPressure);
    ada.setKalmanConfig(kalmanConfig);

    ada.update(ref.refPressure);
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

            EventBroker::getInstance().post(ADA_READY, TOPIC_ADA);
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
        case ADA_RESET:
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

            auto shadowModeDelay =
                getModule<AlgoReference>()->computeTimeSinceLiftoff(
                    shadowModeTime);

            shadowModeTimeoutEvent = EventBroker::getInstance().postDelayed(
                ADA_SHADOW_MODE_TIMEOUT, TOPIC_ADA,
                milliseconds{shadowModeDelay}.count());
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
