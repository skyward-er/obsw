/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro, Riccardo Sironi
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

#include <Main/Configs/NASConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <Main/Configs/ZVKConfig.h>
#include <algorithms/StateInitializer/StateInitializer.h>
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
    : FSM{&ZVKController::state_init, STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::ZVK_PRIORITY}
{
    EventBroker::getInstance().subscribe(this, TOPIC_ZVK);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

ZVKOut ZVKController::getZVKOut()
{
    Lock<FastMutex> lock{zvkMutex};
    return zvkOut;
}

Vector3f ZVKController::getAccHBias()
{
    Lock<FastMutex> lock{zvkMutex};
    return {zvkOut.AccHBias[0], zvkOut.AccHBias[1], zvkOut.AccHBias[2]};
}

Vector3f ZVKController::getAccLBias()
{
    Lock<FastMutex> lock{zvkMutex};
    return {zvkOut.AccLBias[0], zvkOut.AccLBias[1], zvkOut.AccLBias[2]};
}

Vector3f ZVKController::getAccVN100Bias()
{
    Lock<FastMutex> lock{zvkMutex};
    return {zvkOut.AccVN100Bias[0], zvkOut.AccVN100Bias[1],
            zvkOut.AccVN100Bias[2]};
}

Vector3f ZVKController::getGyroHBias()
{
    Lock<FastMutex> lock{zvkMutex};
    return {zvkOut.GyroHBias[0], zvkOut.GyroHBias[1], zvkOut.GyroHBias[2]};
}

Vector3f ZVKController::getGyroLBias()
{
    Lock<FastMutex> lock{zvkMutex};
    return {zvkOut.GyroLBias[0], zvkOut.GyroLBias[1], zvkOut.GyroLBias[2]};
}

Vector3f ZVKController::getGyroVN100Bias()
{
    Lock<FastMutex> lock{zvkMutex};
    return {zvkOut.GyroVN100Bias[0], zvkOut.GyroVN100Bias[1],
            zvkOut.GyroVN100Bias[2]};
}

bool ZVKController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getZvkScheduler();

    zvkTaskId =
        scheduler.addTask([this]() { update(); }, Config::ZVK::UPDATE_RATE);

    if (zvkTaskId == 0)
    {
        LOG_ERR(logger, "Failed to add ZVK update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start ZVK FSM");
        return false;
    }

    zvk.initialize();

    return true;
}

void ZVKController::calibrate()
{
    Lock<FastMutex> lock{zvkMutex};

    // Reinitialize state
    zvk.initialize();

    // calculate initial attitude quaternions
    Sensors* sensors = getModule<Sensors>();
    Vector3f accAcc  = Vector3f::Zero();
    Vector3f magAcc  = Vector3f::Zero();

    // First sample and average the data over a number of samples
    for (int i = 0; i < Config::NAS::CALIBRATION_SAMPLES_COUNT; i++)
    {
        auto imuData = sensors->getIMULastSample();
        auto magData = sensors->getCalibratedLIS2MDLRcsLastSample();

        Vector3f acc = static_cast<AccelerometerData>(imuData);
        Vector3f mag = static_cast<MagnetometerData>(magData);

        accAcc += acc;
        magAcc += mag;

        Thread::sleep(Config::NAS::CALIBRATION_SLEEP_TIME);
    }

    accAcc /= Config::NAS::CALIBRATION_SAMPLES_COUNT;
    accAcc.normalize();
    magAcc /= Config::NAS::CALIBRATION_SAMPLES_COUNT;
    magAcc.normalize();

    // Use the triad to compute initial state
    StateInitializer init;
    Eigen::Vector4f quat = init.triad(accAcc, magAcc, ReferenceConfig::nedMag);

    zvk.setAttitude_Quaternion(quat.data());
}

ZVKControllerState ZVKController::getState() { return state; }

void ZVKController::update()
{
    Lock<FastMutex> lock{zvkMutex};

    if (state.load() == ZVKControllerState::ACTIVE)
    {
        Sensors* sensors = getModule<Sensors>();

        LSM6DSRXData imu0;
        LSM6DSRXData imu1;
        VN100SpiData vn100;

#if defined(DUAL_LSM6)
        imu0 = sensors->getLSM6DSRXHighLastSample();
        imu1 = sensors->getLSM6DSRXLowLastSample();
#else
        vn100 = sensors->getVN100LastSample();
#endif
        ZVKIn input{
            .Accelerometer1Timestamp = imu0.accelerationTimestamp,
            .Accelerometer1Measure   = {imu0.accelerationX, imu0.accelerationY,
                                        imu0.accelerationZ},
            .Accelerometer2Timestamp = imu1.accelerationTimestamp,
            .Accelerometer2Measure   = {imu1.accelerationX, imu1.accelerationY,
                                        imu1.accelerationZ},
            .AccelerometerVN100Timestamp = vn100.accelerationTimestamp,
            .AccelerometerVN100Measure   = {vn100.accelerationX,
                                            vn100.accelerationY,
                                            vn100.accelerationZ},
            .Gyroscope1Timestamp         = imu0.angularSpeedTimestamp,
            .Gyroscope1Measure       = {imu0.angularSpeedX, imu0.angularSpeedY,
                                        imu0.angularSpeedZ},
            .Gyroscope2Timestamp     = imu1.angularSpeedTimestamp,
            .Gyroscope2Measure       = {imu1.angularSpeedX, imu1.angularSpeedY,
                                        imu1.angularSpeedZ},
            .GyroscopeVN100Timestamp = vn100.angularSpeedTimestamp,
            .GyroscopeVN100Measure = {vn100.angularSpeedX, vn100.angularSpeedY,
                                      vn100.angularSpeedZ}};

        zvk.setZVK_In(input);

        zvk.step();

        zvkOut = zvk.getZVK_Out();

        ZVKLogsData logs{TimestampTimer::getTimestamp(),
                         zvk.getZVK_Logs_OBSW()};
        sdLogger.log(logs);
    }
}

void ZVKController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ZVKControllerState::INIT);

            break;
        }

        case ZVK_CALIBRATE:
        {
            transition(&ZVKController::state_calibrating);
            break;
        }
    }
}

void ZVKController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(ZVKControllerState::CALIBRATING);
            calibrate();
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
            transition(&ZVKController::state_calibrating);
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

