/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "NASController.h"

#include <Main/BoardScheduler.h>
#include <Main/Configs/NASConfig.h>
#include <Main/Sensors/Sensors.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace Boardcore;
using namespace Eigen;
using namespace Common;

namespace Main
{

bool NASController::start()
{
    BoardScheduler::getInstance().getScheduler().addTask(
        std::bind(&NASController::update, this), NASConfig::UPDATE_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return ActiveObject::start();
}

void NASController::update()
{
    auto imuData = Sensors::getInstance().getBMX160WithCorrectionLastSample();

    Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                          imuData.accelerationZ);
    Vector3f angularVelocity(imuData.angularVelocityX, imuData.angularVelocityY,
                             imuData.angularVelocityZ);
    Vector3f magneticField(imuData.magneticFieldX, imuData.magneticFieldY,
                           imuData.magneticFieldZ);

    // Predict step
    nas.predictGyro(angularVelocity);
    // nas.predictAcc(acceleration);

    // Correct step
    nas.correctMag(magneticField.normalized());
    nas.correctAcc(acceleration.normalized());
}

NASControllerStatus NASController::getStatus() { return status; }

NASState NASController::getNasState() { return nas.getState(); }

void NASController::setReferenceValues(const ReferenceValues reference)
{
    nas.setReferenceValues(reference);
}

ReferenceValues NASController::getReferenceValues()
{
    return nas.getReferenceValues();
}

void NASController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::IDLE);
        }
        case NAS_CALIBRATE:
        {
            return transition(&NASController::state_calibrating);
        }
    }
}

void NASController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::CALIBRATING);
        }
        case NAS_READY:
        {
            return transition(&NASController::state_ready);
        }
    }
}

void NASController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::READY);
        }
        case NAS_CALIBRATE:
        {
            return transition(&NASController::state_calibrating);
        }
        case FLIGHT_LIFTOFF:
        {
            return transition(&NASController::state_active);
        }
    }
}

void NASController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::ACTIVE);
        }
        case FLIGHT_LANDING_DETECTED:
        {
            return transition(&NASController::state_end);
        }
    }
}

void NASController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::END);
        }
    }
}

void NASController::logStatus(NASControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

NASController::NASController()
    : FSM(&NASController::state_idle), nas(NASConfig::config)
{
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);

    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas.setX(x);
}

NASController::~NASController()
{
    EventBroker::getInstance().unsubscribe(this);
}

}  // namespace Main
