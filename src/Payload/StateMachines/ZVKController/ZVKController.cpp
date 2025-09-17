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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/ZVKConfig.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <algorithm>

using namespace Payload;
using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Eigen;

ZVKController::ZVKController()
    : FSM(&ZVKController::state_init, STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::zvkControllerPriority()),
      zvk{Config::ZVK::CONFIG}
{
    EventBroker::getInstance().subscribe(this, TOPIC_ZVK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool ZVKController::start()
{
    // Initialize state
    Matrix<float, 24, 1> x = Matrix<float, 24, 1>::Zero();
    zvk.setX(x);

    return true;
}

void ZVKController::reset()
{
    Lock<FastMutex> lock{zvkMutex};

    // Reinitialize state
    Matrix<float, 24, 1> x = Matrix<float, 24, 1>::Zero();
    zvk.setX(x);
}

ZVKState ZVKController::getZVKState()
{
    Lock<FastMutex> lock{zvkMutex};
    return zvk.getState();
}

void ZVKController::update()
{
    ZVKControllerState curState = state;

    Lock<FastMutex> lock{zvkMutex};

    if (curState == ZVKControllerState::ACTIVE)
    {
        Sensors* sensors = getModule<Sensors>();

        LSM6DSRXData imu0 = sensors->getLSM6DSRX0LastSample();
        LSM6DSRXData imu1 = sensors->getLSM6DSRX1LastSample();

        zvk.predict();

        zvk.correctZeroVel();

        if (lastAccTimestamp0 < imu0.accelerationTimestamp)
            zvk.correctAcc0(imu0);

        if (lastAccTimestamp1 < imu1.accelerationTimestamp)
            zvk.correctAcc1(imu1);

        if (lastGyroTimestamp0 < imu0.angularSpeedTimestamp)
            zvk.correctGyro0(imu0);

        if (lastGyroTimestamp1 < imu1.angularSpeedTimestamp)
            zvk.correctGyro1(imu1);

        lastAccTimestamp0  = imu0.accelerationTimestamp;
        lastAccTimestamp1  = imu1.accelerationTimestamp;
        lastGyroTimestamp0 = imu0.angularSpeedTimestamp;
        lastGyroTimestamp1 = imu1.angularSpeedTimestamp;

        auto state = zvk.getState();

        sdLogger.log(ZVKState(state));
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
        case ZVK_RESET:
        {
            reset();
            transition(&ZVKController::state_init);
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

