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

#include "NASController.h"

#include <Main/Configs/NASConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <algorithms/NAS/StateInitializer.h>
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

NASController::NASController()
    : FSM{&NASController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::NAS_PRIORITY},
      nas{Config::NAS::CONFIG}
{
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool NASController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getNasScheduler();

    size_t result =
        scheduler.addTask([this]() { update(); }, Config::NAS::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add NAS update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start NAS FSM");
        return false;
    }

    // Initialize reference
    ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();
    nas.setReferenceValues(ref);

    // Initialize state
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();
    Vector4f q             = SkyQuaternion::eul2quat({0, 0, 0});

    x(6) = q(0);
    x(7) = q(1);
    x(8) = q(2);
    x(9) = q(3);

    nas.setX(x);

    return true;
}

NASControllerState NASController::getState() { return state; }

NASState NASController::getNASState()
{
    Lock<FastMutex> lock{nasMutex};
    return nas.getState();
}

void NASController::setOrientation(Eigen::Quaternion<float> quat)
{
    // Need to lock mutex because the only invocation comes from the radio
    // which is a separate thread
    Lock<FastMutex> lock{nasMutex};

    Matrix<float, 13, 1> x = nas.getX();
    x(6)                   = quat.x();
    x(7)                   = quat.y();
    x(8)                   = quat.z();
    x(9)                   = quat.w();
    nas.setX(x);
}

void NASController::update()
{
    NASControllerState curState = state;

    Lock<FastMutex> lock{nasMutex};

    if (curState == NASControllerState::ACTIVE)
    {
        Sensors* sensors = getModule<Sensors>();

        IMUData imu              = sensors->getIMULastSample();
        UBXGPSData gps           = sensors->getUBXGPSLastSample();
        PressureData baro        = sensors->getAtmosPressureLastSample();
        PressureData staticPitot = sensors->getCanPitotStaticPressLastSample();
        PressureData dynamicPitot =
            sensors->getCanPitotDynamicPressLastSample();

        // Calculate acceleration
        Vector3f acc    = static_cast<AccelerometerData>(imu);
        float accLength = acc.norm();

        // Perform initial NAS prediction
        // TODO: What about stale data?
        nas.predictGyro(imu);
        nas.predictAcc(imu);

        // Then perform necessary corrections
        // Disable magnetometer correction
        // if (lastMagTimestamp < imu.magneticFieldTimestamp &&
        //     magDecimateCount == Config::NAS::MAGNETOMETER_DECIMATE)
        // {
        //     nas.correctMag(imu);
        //     magDecimateCount = 0;
        // }
        // else
        // {
        //     magDecimateCount++;
        // }

        if (lastGpsTimestamp < gps.gpsTimestamp && gps.fix == 3 &&
            accLength < Config::NAS::DISABLE_GPS_ACCELERATION)
        {
            nas.correctGPS(gps);
        }

        if (lastBaroTimestamp < baro.pressureTimestamp)
            nas.correctBaro(baro.pressure);

        // Correct with pitot if one pressure sample is new
        // Disable pitot correction
        // if (dynamicPitot.pressure > 0 &&
        //     (staticPitotTimestamp < staticPitot.pressureTimestamp ||
        //      dynamicPitotTimestamp < dynamicPitot.pressureTimestamp) &&
        //     (-nas.getState().d < Config::NAS::PITOT_ALTITUDE_THRESHOLD) &&
        //     (-nas.getState().vd > Config::NAS::PITOT_SPEED_THRESHOLD))
        // {
        //     nas.correctPitot(staticPitot.pressure, dynamicPitot.pressure);
        // }

        // Correct with accelerometer if the acceleration is in specs
        if (lastAccTimestamp < imu.accelerationTimestamp && acc1g)
            nas.correctAcc(imu);

        // Check if the accelerometer is measuring 1g
        if (accLength <
                (Constants::g + Config::NAS::ACCELERATION_1G_CONFIDENCE / 2) &&
            accLength >
                (Constants::g - Config::NAS::ACCELERATION_1G_CONFIDENCE / 2))
        {
            if (acc1gSamplesCount < Config::NAS::ACCELERATION_1G_SAMPLES)
                acc1gSamplesCount++;
            else
                acc1g = true;
        }
        else
        {
            acc1gSamplesCount = 0;
            acc1g             = false;
        }

        lastGyroTimestamp     = imu.angularSpeedTimestamp;
        lastAccTimestamp      = imu.accelerationTimestamp;
        lastMagTimestamp      = imu.magneticFieldTimestamp;
        lastGpsTimestamp      = gps.gpsTimestamp;
        lastBaroTimestamp     = baro.pressureTimestamp;
        staticPitotTimestamp  = staticPitot.pressureTimestamp;
        dynamicPitotTimestamp = dynamicPitot.pressureTimestamp;

        auto state = nas.getState();

        getModule<StatsRecorder>()->updateNas(state);
        sdLogger.log(state);
    }
}

void NASController::calibrate()
{
    Sensors* sensors = getModule<Sensors>();

    Vector3f accAcc = Vector3f::Zero();
    Vector3f magAcc = Vector3f::Zero();

    // First sample and average the data over a number of samples
    for (int i = 0; i < Config::NAS::CALIBRATION_SAMPLES_COUNT; i++)
    {
        IMUData imu = sensors->getIMULastSample();

        Vector3f acc = static_cast<AccelerometerData>(imu);
        Vector3f mag = static_cast<MagnetometerData>(imu);

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
    init.triad(accAcc, magAcc, ReferenceConfig::nedMag);

    ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();

    Lock<FastMutex> lock{nasMutex};
    nas.setX(init.getInitX());
    nas.setReferenceValues(ref);

    EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
}

void NASController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::INIT);
            break;
        }

        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }
    }
}

void NASController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::CALIBRATING);
            calibrate();
            break;
        }

        case NAS_READY:
        {
            transition(&NASController::state_ready);
            break;
        }
    }
}

void NASController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::READY);
            break;
        }

        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }

        case NAS_FORCE_START:
        case FLIGHT_ARMED:
        {
            transition(&NASController::state_active);
            break;
        }
    }
}

void NASController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::ACTIVE);
            break;
        }
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&NASController::state_end);
            break;
        }
        case NAS_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            transition(&NASController::state_ready);
            break;
        }
    }
}

void NASController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::END);
            break;
        }
    }
}

void NASController::updateAndLogStatus(NASControllerState state)
{
    this->state              = state;
    NASControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
