/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/MEAConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <functional>

using namespace Boardcore;
using namespace Common;
using namespace std;

namespace Main
{
MEAController::MEAController(TaskScheduler* sched)
    : FSM(&MEAController::state_idle), mea(getMEAKalmanConfig()),
      scheduler(sched)
{
    // Subscribe the class of the topics
    EventBroker::getInstance().subscribe(this, TOPIC_MEA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool MEAController::start()
{
    size_t result = scheduler->addTask(bind(&MEAController::update, this),
                                       MEAConfig::UPDATE_PERIOD,
                                       TaskScheduler::Policy::RECOVER);
    return ActiveObject::start() && result != 0;
}

void MEAController::update()
{
    ModuleManager& modules  = ModuleManager::getInstance();
    PressureData ccPressure = modules.get<Sensors>()->getCCPressureLastSample();

    // No need for pause kernel due to its presence inside the getter
    MEAControllerStatus status = getStatus();
    NASState nasState          = modules.get<NASController>()->getNasState();

    // Get mach number and estimated CD
    float mach = computeMach(nasState);
    float CD   = computeCD(mach);
    float rho  = computeRho(nasState);

    // Get MAIN valve state from CAN bus
    float valvePosition =
        modules.get<Actuators>()->getServoPosition(ServosList::MAIN_VALVE) > 0.3
            ? 1
            : 0;

    // If the state is active (or armed) and the pressure is greater than the
    // threshold update the kalman
    switch (status.state)
    {
        case MEAControllerState::ARMED:
        {
            if (ccPressure.pressure >= MEAConfig::CC_PRESSURE_THRESHOLD &&
                ccPressure.pressureTimestamp > lastUpdateTimestamp)
            {
                mea.update(valvePosition, ccPressure.pressure);
                lastUpdateTimestamp = TimestampTimer::getTimestamp();
            }
            break;
        }
        case MEAControllerState::SHADOW_MODE:
        {
            if (ccPressure.pressure >= MEAConfig::CC_PRESSURE_THRESHOLD &&
                ccPressure.pressureTimestamp > lastUpdateTimestamp)
            {
                mea.update(valvePosition, ccPressure.pressure);
                lastUpdateTimestamp = TimestampTimer::getTimestamp();
            }

            // Compute the estimated altitude
            estimatedAltitude =
                computeAltitude(nasState, mea.getState().x2, CD, 0.15f, rho);
            estimatedMass = mea.getState().x2;

            if (estimatedAltitude >= MEAConfig::SHUTDOWN_THRESHOLD_ALTITUDE)
            {
                detectedShutdowns++;
            }
            else
            {
                // Shutdowns must be consecutive in order to be valid
                detectedShutdowns = 0;
            }

            logStatus(status.state);
            break;
        }
        case MEAControllerState::ACTIVE:
        {
            if (ccPressure.pressure >= MEAConfig::CC_PRESSURE_THRESHOLD &&
                ccPressure.pressureTimestamp > lastUpdateTimestamp)
            {
                mea.update(valvePosition, ccPressure.pressure);
                lastUpdateTimestamp = TimestampTimer::getTimestamp();
            }

            // Compute the estimated altitude
            estimatedAltitude =
                computeAltitude(nasState, mea.getState().x2, CD, 0.15f, rho);
            estimatedMass = mea.getState().x2;

            if (estimatedAltitude >= MEAConfig::SHUTDOWN_THRESHOLD_ALTITUDE)
            {
                detectedShutdowns++;
            }
            else
            {
                // Shutdowns must be consecutive in order to be valid
                detectedShutdowns = 0;
            }

            if (detectedShutdowns > MEAConfig::SHUTDOWN_N_SAMPLES)
            {
                EventBroker::getInstance().post(MEA_SHUTDOWN_DETECTED,
                                                TOPIC_MEA);
            }

            logStatus(status.state);
            break;
        }
        case MEAControllerState::ACTIVE_DISARMED:
        {
            if (ccPressure.pressure >= MEAConfig::CC_PRESSURE_THRESHOLD &&
                ccPressure.pressureTimestamp > lastUpdateTimestamp)
            {
                mea.update(valvePosition, ccPressure.pressure);
                lastUpdateTimestamp = TimestampTimer::getTimestamp();
            }

            estimatedMass = mea.getState().x2;

            logStatus(status.state);
            break;
        }
        default:
        {
            break;
        }
    }

    Logger::getInstance().log(getMEAState());
}

MEAControllerStatus MEAController::getStatus()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return status;
}

MEAState MEAController::getMEAState()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return mea.getState();
}

void MEAController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(MEAControllerState::IDLE);

            return transition(&MEAController::state_ready);
        }
    }
}

void MEAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(MEAControllerState::READY);
        }
        case FLIGHT_ARMED:
        {
            return transition(&MEAController::state_armed);
        }
    }
}

void MEAController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(MEAControllerState::ARMED);
        }
        case FLIGHT_DISARMED:
        {
            return transition(&MEAController::state_ready);
        }
        case FLIGHT_LIFTOFF:
        {
            return transition(&MEAController::state_shadow_mode);
        }
    }
}

void MEAController::state_shadow_mode(const Event& event)
{
    static uint16_t shadowModeTimeoutEventId = 0;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(MEAControllerState::SHADOW_MODE);

            // Add a delayed event to exit the shadow mode
            shadowModeTimeoutEventId = EventBroker::getInstance().postDelayed(
                MOTOR_SHADOW_MODE_TIMEOUT, TOPIC_MEA,
                MEAConfig::SHADOW_MODE_TIMEOUT);
            break;
        }
        case EV_EXIT:
        {
            // Remove the shadow mode event. This works even though the event is
            // expired (aka after shadow_mode_timeout) because the event broker
            // assigns a progressive number for every delayed event. If and only
            // if the number of registered delayed event is less than 2^16, then
            // this technique is valid.
            return EventBroker::getInstance().removeDelayed(
                shadowModeTimeoutEventId);
        }
        case MOTOR_SHADOW_MODE_TIMEOUT:
        {
            return transition(&MEAController::state_active);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&MEAController::state_end);
        }
    }
}

void MEAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // Zero the number of so far detected shutdowns. It is thread safe
            // due to std::atomic variable
            detectedShutdowns = 0;
            return logStatus(MEAControllerState::ACTIVE);
        }
        case FLIGHT_MOTOR_SHUTDOWN:
        {
            return transition(&MEAController::state_active_disarmed);
        }
        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&MEAController::state_end);
        }
    }
}

void MEAController::state_active_disarmed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(MEAControllerState::ACTIVE_DISARMED);
        }
        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&MEAController::state_end);
        }
    }
}

void MEAController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(MEAControllerState::END);
        }
    }
}

void MEAController::logStatus(MEAControllerState state)
{
    {
        miosix::PauseKernelLock lock;
        status.timestamp         = TimestampTimer::getTimestamp();
        status.state             = state;
        status.detectedShutdowns = detectedShutdowns;
        status.estimatedApogee   = estimatedAltitude;
        status.estimatedMass     = estimatedMass;
    }

    Logger::getInstance().log(status);
}

float MEAController::computeMach(NASState state)
{
    float T = Constants::MSL_TEMPERATURE + Constants::a * (-state.d);
    float c = sqrt(1.4f * Constants::R * T);
    return -state.vd / c;
}

float MEAController::computeCD(float mach)
{
    return MEAConfig::n000 + MEAConfig::n100 * mach +
           MEAConfig::n200 * powf(mach, 2) + MEAConfig::n300 * powf(mach, 3) +
           MEAConfig::n400 * powf(mach, 4) + MEAConfig::n500 * powf(mach, 5) +
           MEAConfig::n600 * powf(mach, 6);
}

float MEAController::computeAltitude(NASState state, float mass, float CD,
                                     float D, float rho)
{
    // Compute rocket surface
    float S = Constants::PI * (D / 2) * (D / 2);

    return -state.d +
           1 / (2 * (0.5 * rho * CD * S / mass)) *
               log1p(((-state.vd) * (-state.vd) * (0.5 * rho * CD * S) / mass) /
                     Constants::g);
}

float MEAController::computeRho(NASState state)
{
    return Constants::RHO_0 *
           exp(state.d / 11000.f);  // 11000 is the troposphere height
}

MEA::KalmanFilter::KalmanConfig MEAController::getMEAKalmanConfig()
{
    MEA::KalmanFilter::MatrixNN F_INIT;
    MEA::KalmanFilter::MatrixPN H_INIT;
    MEA::KalmanFilter::MatrixNN P_INIT;
    MEA::KalmanFilter::MatrixNN Q_INIT;
    MEA::KalmanFilter::MatrixPP R_INIT;
    MEA::KalmanFilter::MatrixNM G_INIT;

    // clang-format off
    F_INIT = MEA::KalmanFilter::MatrixNN({
            {1.435871191228868, -0.469001276508780,  0.f}, 
            {1.f,                0.f,                0.f},
            {-0.002045309260755, 0.001867496708935,  1.f}});
    
    H_INIT = {1.780138883879285,-1.625379384370081,0.f};

    P_INIT    = MEA::KalmanFilter::MatrixNN::Zero();
    Q_INIT    = MEAConfig::MODEL_NOISE_VARIANCE * MEA::KalmanFilter::CVectorN({1, 1, 1}).asDiagonal();
    R_INIT[0] = MEAConfig::SENSOR_NOISE_VARIANCE;
    G_INIT    = MEA::KalmanFilter::MatrixNM{{4}, {0}, {0}};
    // clang-format on

    return {F_INIT,
            H_INIT,
            Q_INIT,
            R_INIT,
            P_INIT,
            G_INIT,
            MEA::KalmanFilter::CVectorN{
                0, 0, MEAConfig::DEFAULT_INITIAL_ROCKET_MASS}};
}

}  // namespace Main