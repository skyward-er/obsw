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

#include <Parafoil/Configs/ADAConfig.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <events/EventBroker.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <algorithm>
#include <chrono>

using namespace std::chrono;
using namespace Parafoil;
using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Constants;

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
          BoardScheduler::adaControllerPriority()},
      ada0{DEFAULT_KALMAN_CONFIG}
{
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    wingDescentDetected = false;
}

bool ADAController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->adaController();

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

    return true;
}

ADAState ADAController::getADAState()
{
    Lock<FastMutex> lock{adaMutex};

    return ada0.getState();
}

ADAControllerState ADAController::getState() { return state; }

float ADAController::getMaxVerticalSpeed()
{
    return ada0.getState().verticalSpeed;
}

float ADAController::getMaxPressure() { return ada0.getState().x0; }

ADAController::detectedApogees ADAController::getDetectedApogees()
{
    detectedApogees apogees;
    apogees.ada0DetectedApogees = wingDescentDetected ? 1 : 0;
    apogees.ada0DetectedApogees = 0;
    apogees.ada0DetectedApogees = 0;
    return apogees;
}

float ADAController::getVerticalSpeedCov()
{
    ReferenceValues ref  = ada0.getReferenceValues();
    ADAState state       = ada0.getState();
    const float* QMatrix = ada0.getFlatq();
    float n              = 1 / (a * R / g);
    float cov2 = -(ref.refTemperature * pow(state.x0 / ref.refPressure,
                                            (1 / n) / (a * n * state.x0)));
    float cov1 = cov2 * state.x1 / n / state.x0 * (1 - n);
    Eigen::Matrix<float, 1, 2> cov;
    cov << cov1, cov2;
    Eigen::Matrix<float, 2, 2> covariances;
    covariances << QMatrix[0], QMatrix[3], QMatrix[1], QMatrix[4];
    return cov * covariances * cov.transpose();
}

void ADAController::setReferenceValues(const Boardcore::ReferenceValues& ref)
{
    miosix::Lock<miosix::FastMutex> l(adaMutex);
    ada0.setReferenceValues(ref);
}

void ADAController::update()
{
    ADAControllerState curState = state;

    // Lock ADA for the whole duration of the update
    Lock<FastMutex> lock{adaMutex};

    // First update the kalman
    if (curState == ADAControllerState::ARMED ||
        curState == ADAControllerState::ACTIVE)
    {
        PressureData baro0 =
            getModule<Sensors>()->getStaticPressureLastSample();

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
    }

    auto ada0State = ada0.getState();

    // Log this sample step
    ADAControllerSampleData data = {
        TimestampTimer::getTimestamp(), 1, 1, 1, 1, 1, 1, curState};
    sdLogger.log(data);

    // Logs ADA states as separate structs
    sdLogger.log(ADA0State(ada0State));
}

void ADAController::calibrate()
{
    ReferenceValues ref = getModule<NASController>()->getReferenceValues();

    Lock<FastMutex> lock{adaMutex};
    ada0.setReferenceValues(ref);

    // Recomputing Kalman config also resets the ADA state
    auto kalmanConfig = computeADAKalmanConfig(ref.refPressure);
    ada0.setKalmanConfig(kalmanConfig);

    ada0.update(ref.refPressure);
}

const float* ADAController::getQflattened() const { return ada0.getFlatq(); }

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

        case FLIGHT_WING_DESCENT:
        {
            return transition(&ADAController::state_active);
        }

        case FLIGHT_DISARMED:
        {
            transition(&ADAController::state_ready);
            break;
        }
    }
}

void ADAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            wingDescentDetected = true;
            updateAndLogStatus(ADAControllerState::ACTIVE);
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
