/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro
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

#include "ZVKController.h"

#include <Main/Configs/SchedulerConfig.h>
#include <Main/Configs/ZVKConfig.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <algorithm>

using namespace Main;
using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Eigen;

ZVKController::ZVKController()
    : FSM{&ZVKController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::ZVK_PRIORITY},
      zvk{Config::ZVK::CONFIG}
{
    EventBroker::getInstance().subscribe(this, TOPIC_ZVK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool ZVKController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getZvkScheduler();

    size_t result =
        scheduler.addTask([this]() { update(); }, Config::ZVK::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add ZVK update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ZVK FSM");
        return false;
    }

    // Initialize state
    Matrix<float, 16, 1> x = Matrix<float, 16, 1>::Zero();
    Vector4f q = SkyQuaternion::eul2quat(Config::ZVK::initialAttitude);

    x(0) = q(0);
    x(1) = q(1);
    x(2) = q(2);
    x(3) = q(3);

    zvk.setX(x);

    return true;
}

ZVKControllerState ZVKController::getState() { return state; }

ZVKState ZVKController::getZVKState()
{
    Lock<FastMutex> lock{zvkMutex};
    return zvk.getState();
}

void ZVKController::setOrientation(Eigen::Quaternion<float> quat)
{
    Lock<FastMutex> lock{zvkMutex};

    Matrix<float, 16, 1> x = zvk.getX();
    x(0)                   = quat.x();
    x(1)                   = quat.y();
    x(2)                   = quat.z();
    x(3)                   = quat.w();
    zvk.setX(x);
}

void ZVKController::update()
{
    ZVKControllerState curState = state;

    Lock<FastMutex> lock{zvkMutex};

    if (curState == ZVKControllerState::ACTIVE)
    {
        Sensors* sensors = getModule<Sensors>();

        IMUData imu = sensors->getIMULastSample();

        zvk.predict(imu, imu);

        if (lastMagTimestamp < imu.magneticFieldTimestamp)

            zvk.correct(imu, imu, imu);

        lastGyroTimestamp = imu.angularSpeedTimestamp;
        lastAccTimestamp  = imu.accelerationTimestamp;
        lastMagTimestamp  = imu.magneticFieldTimestamp;

        auto state = zvk.getState();

        sdLogger.log(state);
    }
}

void ZVKController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ZVKControllerState::INIT);

            // Immediate transition to active
            transition(&ZVKController::state_active);
            break;
        }
    }
}

void ZVKController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ZVKControllerState::ACTIVE);
            break;
        }
        case FLIGHT_ARMED:
        case ZVK_FORCE_STOP:
        {
            transition(&ZVKController::state_end);
            break;
        }
    }
}

void ZVKController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ZVKControllerState::END);
            break;
        }
    }
}

void ZVKController::updateAndLogStatus(ZVKControllerState state)
{
    this->state              = state;
    ZVKControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}

