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
#include <algorithms/NAS/StateInitializer.h>
#include <common/events/Events.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace std;
using namespace Eigen;
using namespace Boardcore;
using namespace Common;

namespace Main
{

bool NASController::start()
{
    // Add the update task to the scheduler
    BoardScheduler::getInstance().getScheduler().addTask(
        bind(&NASController::update, this), NASConfig::UPDATE_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return ActiveObject::start();
}

void NASController::update()
{
    // If the nas is not active i skip the step
    if (!this->testState(&NASController::state_active))
        return;

    auto imuData = Sensors::getInstance().getBMX160WithCorrectionLastSample();
    auto gpsData = Sensors::getInstance().getUbxGpsLastSample();
    auto pressureData = Sensors::getInstance().getMS5803LastSample();

    // Predict step
    nas.predictGyro(imuData);
    nas.predictAcc(imuData);

    // Correct step
    nas.correctMag(imuData);
    nas.correctAcc(imuData);
    nas.correctGPS(gpsData);
    nas.correctBaro(pressureData.pressure);

    Logger::getInstance().log(nas.getState());

#ifdef HILSimulation
    // useful only for hil testing
    updateData(nas.getState());
#endif  // HILSimulation
}

void NASController::initializeOrientationAndPressure()
{
    // Mean 10 accelerometer values
    Eigen::Vector3f accelerometer;
    Eigen::Vector3f magnetometer;
    float pressure    = 0;
    float temperature = 0;
    StateInitializer state;
    UBXGPSData gps            = Sensors::getInstance().getUbxGpsLastSample();
    ReferenceValues reference = nas.getReferenceValues();

    // Mean the values
    for (int i = 0; i < NASConfig::MEAN_COUNT; i++)
    {
        // IMU
        BMX160WithCorrectionData imuData =
            Sensors::getInstance().getBMX160WithCorrectionLastSample();
        accelerometer +=
            Eigen::Vector3f(imuData.accelerationX, imuData.accelerationY,
                            imuData.accelerationZ);
        magnetometer +=
            Eigen::Vector3f(imuData.magneticFieldX, imuData.magneticFieldY,
                            imuData.magneticFieldZ);

        // Barometer
        MS5803Data pressureData = Sensors::getInstance().getMS5803LastSample();
        pressure += pressureData.pressure;
        temperature += pressureData.temperature;

        // Wait for some time
        miosix::Thread::sleep(100);
    }

    accelerometer /= NASConfig::MEAN_COUNT;
    magnetometer /= NASConfig::MEAN_COUNT;
    pressure /= NASConfig::MEAN_COUNT;
    temperature /= NASConfig::MEAN_COUNT;

    // Normalize the data
    accelerometer.normalize();
    magnetometer.normalize();

    // Triad the initial orientation
    state.triad(accelerometer, magnetometer, NASConfig::nedMag);

    // Set the pressure reference using an already existing reference values
    reference.pressure    = pressure;
    reference.temperature = temperature;

    // If in this moment the GPS has fix i use that position as starting
    if (gps.fix != 0)
    {
        reference.startLatitude  = gps.latitude;
        reference.startLongitude = gps.longitude;
    }

    // Set the values inside the NAS
    {
        miosix::PauseKernelLock l;

        nas.setX(state.getInitX());
        nas.setReferenceValues(reference);
    }

    // At the end i publish on the nas topic the end
    EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
}

void NASController::setCoordinates(Eigen::Vector2f position)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = nas.getReferenceValues();
    reference.startLatitude   = position[0];
    reference.startLongitude  = position[1];
    nas.setReferenceValues(reference);
}

void NASController::setOrientation(float yaw, float pitch, float roll)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    auto x = nas.getX();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat({yaw, pitch, roll});
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas.setX(x);
}

void NASController::setReferenceAltitude(float altitude)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = nas.getReferenceValues();
    reference.altitude        = altitude;
    nas.setReferenceValues(reference);
}

void NASController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = nas.getReferenceValues();
    reference.temperature     = temperature;
    nas.setReferenceValues(reference);
}

void NASController::setReferenceValues(const ReferenceValues reference)
{
    miosix::PauseKernelLock l;
    nas.setReferenceValues(reference);
}

NASControllerStatus NASController::getStatus() { return status; }

NASState NASController::getNasState()
{
    miosix::PauseKernelLock l;
    return nas.getState();
}

ReferenceValues NASController::getReferenceValues()
{
    return nas.getReferenceValues();
}

void NASController::state_idle(const Event &event)
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

void NASController::state_calibrating(const Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            // Calibrate the NAS
            initializeOrientationAndPressure();

            return logStatus(NASControllerState::CALIBRATING);
        }
        case NAS_READY:
        {
            return transition(&NASController::state_ready);
        }
    }
}

void NASController::state_ready(const Event &event)
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
        case FLIGHT_ARMED:
        {
            return transition(&NASController::state_active);
        }
    }
}

void NASController::state_active(const Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::ACTIVE);
        }
        case FLIGHT_LANDING_DETECTED:
        case FLIGHT_MISSION_TIMEOUT:
        {
            return transition(&NASController::state_end);
        }
    }
}

void NASController::state_end(const Event &event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            return logStatus(NASControllerState::END);
        }
    }
}

#ifdef HILSimulation
void NASController::setUpdateDataFunction(
    function<void(Boardcore::NASState)> updateData)
{
    this->updateData = updateData;
}
#endif

void NASController::logStatus(NASControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

NASController::NASController()
    : FSM(&NASController::state_idle), nas(NASConfig::config)
#ifdef HILSimulation
      ,
      updateData([](Boardcore::NASState) {})
#endif
{
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);

    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas.setX(x);
    nas.setReferenceValues(NASConfig::defaultReferenceValues);
}

NASController::~NASController()
{
    EventBroker::getInstance().unsubscribe(this);
}

}  // namespace Main
