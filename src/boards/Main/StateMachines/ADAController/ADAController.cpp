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

#include <Main/Configs/ADAConfig.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <functional>

using namespace Boardcore;
using namespace Common;
using namespace std;

namespace Main
{
ADAController::ADAController(TaskScheduler* sched)
    : FSM(&ADAController::state_idle), ada(getADAKalmanConfig()),
      scheduler(sched)
{
    // Subscribe the class to the topics
    EventBroker::getInstance().subscribe(this, TOPIC_ADA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);

    // Set the default reference values
    ada.setReferenceValues(ReferenceConfig::defaultReferenceValues);
}

bool ADAController::start()
{
    // Add the task to the scheduler
    size_t result = scheduler->addTask(bind(&ADAController::update, this),
                                       ADAConfig::UPDATE_PERIOD,
                                       TaskScheduler::Policy::RECOVER);

    return ActiveObject::start() && result != 0;
}

void ADAController::update()
{
    ModuleManager& modules = ModuleManager::getInstance();
    PressureData barometerData =
        modules.get<Sensors>()->getStaticPressure1LastSample();

    // Get a snapshot of the situation. There is no need to synchronize because
    // the getter are already thread safe with a PauseKernel
    ADAControllerStatus status = getStatus();
    ADAState state             = getADAState();

    // The algorithm changes its actions depending on the FSM state
    switch (status.state)
    {
        case ADAControllerState::ARMED:
        {
            ada.update(barometerData.pressure);
        }
        case ADAControllerState::SHADOW_MODE:
        {
            // During shadow-mode no event will be thrown
            ada.update(barometerData.pressure);

            // Check for apogees
            if (state.verticalSpeed < ADAConfig::APOGEE_VERTICAL_SPEED_TARGET)
            {
                detectedApogees++;
            }
            else
            {
                // Apogees must be consecutive in order to be valid
                detectedApogees = 0;
            }

            // Log the detected apogees during this phase
            logStatus(status.state);

            break;
        }
        case ADAControllerState::ACTIVE:
        {
            // During shadow-mode no event will be thrown
            ada.update(barometerData.pressure);

            // Check for apogees
            if (state.verticalSpeed < ADAConfig::APOGEE_VERTICAL_SPEED_TARGET)
            {
                detectedApogees++;
            }
            else
            {
                // Apogees must be consecutive in order to be valid
                detectedApogees = 0;
            }

            // Check if the number of apogees has reached the limit
            if (detectedApogees > ADAConfig::APOGEE_N_SAMPLES)
            {
                EventBroker::getInstance().post(ADA_APOGEE_DETECTED, TOPIC_ADA);
            }

            // Log the detected apogees during this phase
            logStatus(status.state);

            break;
        }
        default:
        {
            break;
        }
    }
    Logger::getInstance().log(getADAState());
}

void ADAController::calibrate()
{
    Stats pressure;
    ModuleManager& modules = ModuleManager::getInstance();

    for (int i = 0; i < ADAConfig::CALIBRATION_SAMPLES_COUNT; i++)
    {
        PressureData data =
            modules.get<Sensors>()->getStaticPressure1LastSample();
        pressure.add(data.pressure);

        miosix::Thread::sleep(ADAConfig::CALIBRATION_SLEEP_TIME);
    }

    // Set the pressure and temperature reference
    ReferenceValues reference = ada.getReferenceValues();
    reference.refPressure     = pressure.getStats().mean;

    // clang-format off
    reference.refAltitude     = Aeroutils::relAltitude(reference.refPressure, 
                                                       reference.mslPressure, 
                                                       reference.mslTemperature);
    // clang-format on

    // Update the algorithm reference values
    {
        miosix::PauseKernelLock l;
        ada.setReferenceValues(reference);
        ada.setKalmanConfig(getADAKalmanConfig());
        ada.update(reference.refPressure);
    }

    EventBroker::getInstance().post(ADA_READY, TOPIC_ADA);
}

void ADAController::setReferenceAltitude(float altitude)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = ada.getReferenceValues();
    reference.refAltitude     = altitude;
    ada.setReferenceValues(reference);
}

void ADAController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = ada.getReferenceValues();

    // The temperature is in degrees, converted in kelvin
    reference.refTemperature = temperature + 273.15f;
    ada.setReferenceValues(reference);
}

void ADAController::setReferenceValues(const ReferenceValues reference)
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    ada.setReferenceValues(reference);
}

ADAControllerStatus ADAController::getStatus()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return status;
}

ADAState ADAController::getADAState()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return ada.getState();
}

ReferenceValues ADAController::getReferenceValues()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return ada.getReferenceValues();
}

void ADAController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::IDLE);
        }
        case ADA_CALIBRATE:
        {
            return transition(&ADAController::state_calibrating);
        }
    }
}

void ADAController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ADAControllerState::CALIBRATING);

            // Calibrate the ADA
            calibrate();
            break;
        }
        case ADA_READY:
        {
            return transition(&ADAController::state_ready);
        }
    }
}

void ADAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::READY);
        }
        case ADA_CALIBRATE:
        {
            return transition(&ADAController::state_calibrating);
        }
        case ADA_FORCE_START:
        {
            // Skip directly to the active/shadow mode mode
            return transition(&ADAController::state_shadow_mode);
        }
        case FLIGHT_ARMED:
        {
            return transition(&ADAController::state_armed);
        }
    }
}

void ADAController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::ARMED);
        }
        case FLIGHT_DISARMED:
        {
            return transition(&ADAController::state_ready);
        }
        case FLIGHT_LIFTOFF:
        {
            return transition(&ADAController::state_shadow_mode);
        }
    }
}

void ADAController::state_shadow_mode(const Event& event)
{
    static uint16_t shadowModeTimeoutEventId = 0;

    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(ADAControllerState::SHADOW_MODE);

            // Add a delayed event to exit the shadow mode
            shadowModeTimeoutEventId = EventBroker::getInstance().postDelayed(
                ADA_SHADOW_MODE_TIMEOUT, TOPIC_ADA,
                ADAConfig::SHADOW_MODE_TIMEOUT);
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
        case ADA_SHADOW_MODE_TIMEOUT:
        {
            return transition(&ADAController::state_active);
        }
        case ADA_FORCE_STOP:
        {
            return transition(&ADAController::state_ready);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&ADAController::state_end);
        }
    }
}

void ADAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // Zero the number of so far detected apogees. It is thread safe due
            // to std::atomic variable
            detectedApogees = 0;
            return logStatus(ADAControllerState::ACTIVE);
        }
        case ADA_FORCE_STOP:
        {
            return transition(&ADAController::state_ready);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&ADAController::state_end);
        }
    }
}

void ADAController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(ADAControllerState::END);
        }
    }
}

void ADAController::logStatus(ADAControllerState state)
{
    {
        miosix::PauseKernelLock lock;
        // Update the current FSM state
        status.timestamp       = TimestampTimer::getTimestamp();
        status.state           = state;
        status.detectedApogees = detectedApogees;
    }

    // Log the status
    Logger::getInstance().log(status);
}

ADA::KalmanFilter::KalmanConfig ADAController::getADAKalmanConfig()
{
    ADA::KalmanFilter::MatrixNN F_INIT;
    ADA::KalmanFilter::MatrixPN H_INIT;
    ADA::KalmanFilter::MatrixNN P_INIT;
    ADA::KalmanFilter::MatrixNN Q_INIT;
    ADA::KalmanFilter::MatrixPP R_INIT;
    ADA::KalmanFilter::MatrixNM G_INIT;

    // clang-format off
    F_INIT  = ADA::KalmanFilter::MatrixNN({
        {1.0, ADAConfig::SAMPLING_PERIOD,    0.5f * ADAConfig::SAMPLING_PERIOD * ADAConfig::SAMPLING_PERIOD},
        {0.0, 1.0,                           ADAConfig::SAMPLING_PERIOD},
        // cppcheck-suppress constStatement
        {0.0, 0.0,                           1.0}});
    // clang-format on

    H_INIT = {1.0, 0.0, 0.0};

    // cppcheck-suppress constStatement
    P_INIT = ADA::KalmanFilter::MatrixNN(
        {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});

    // cppcheck-suppress constStatement
    Q_INIT << ADA::KalmanFilter::MatrixNN(
        {{30.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 2.5f}});

    R_INIT[0] = 4000.0f;

    G_INIT = ADA::KalmanFilter::MatrixNM::Zero();

    return {F_INIT,
            H_INIT,
            Q_INIT,
            R_INIT,
            P_INIT,
            G_INIT,
            ADA::KalmanFilter::CVectorN(ada.getReferenceValues().refPressure, 0,
                                        0)};
}
}  // namespace Main