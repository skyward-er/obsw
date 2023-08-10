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
#include <Main/StateMachines/ADAController/ADAController.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

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

void ADAController::update() {}

void ADAController::calibrate() {}

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

void ADAController::state_idle(const Event& event) {}

void ADAController::state_calibrating(const Event& event) {}

void ADAController::state_ready(const Event& event) {}

void ADAController::state_shadow_mode(const Event& event) {}

void ADAController::state_active(const Event& event) {}

void ADAController::state_end(const Event& event) {}

void ADAController::logStatus(ADAControllerState state)
{
    {
        miosix::PauseKernelLock lock;
        // Update the current FSM state
        status.timestamp = TimestampTimer::getTimestamp();
        status.state     = state;
    }

    // Log the status
    Logger::getInstance().log(status);
}

ADA::KalmanFilter::KalmanConfig ADAController::getADAKalmanConfig()
{
    ADA::KalmanFilter::MatrixNN F_INIT;
    // clang-format off
    F_INIT <<
        1.0, ADAConfig::SAMPLING_PERIOD, 0.5f * ADAConfig::SAMPLING_PERIOD * ADAConfig::SAMPLING_PERIOD,
        0.0, 1.0,             ADAConfig::SAMPLING_PERIOD,
        // cppcheck-suppress constStatement
        0.0, 0.0,             1.0;
    // clang-format on
    ADA::KalmanFilter::MatrixPN H_INIT{1.0, 0.0, 0.0};
    ADA::KalmanFilter::MatrixNN P_INIT;
    // cppcheck-suppress constStatement
    P_INIT << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    ADA::KalmanFilter::MatrixNN Q_INIT;
    // cppcheck-suppress constStatement
    Q_INIT << 30.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 2.5f;
    ADA::KalmanFilter::MatrixPP R_INIT{4000.0f};

    return {F_INIT,
            H_INIT,
            Q_INIT,
            R_INIT,
            P_INIT,
            ADA::KalmanFilter::CVectorN(ada.getReferenceValues().refPressure, 0,
                                        0)};
}
}  // namespace Main