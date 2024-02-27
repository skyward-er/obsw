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

#include <Parafoil/Configs/NASConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <algorithms/NAS/StateInitializer.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace Boardcore;
using namespace Eigen;
using namespace std;
using namespace Common;
namespace Parafoil
{
NASController::NASController(TaskScheduler* sched)
    : FSM(&NASController::state_idle), nas(NASConfig::config), scheduler(sched)
{
    // Subscribe the class to the topics
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);

    // Setup the NAS
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Create the initial quaternion
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});

    // Set the initial quaternion inside the matrix
    x(6) = q(0);
    x(7) = q(1);
    x(8) = q(2);
    x(9) = q(3);

    // Set the NAS x matrix
    nas.setX(x);

    // Set the referenced values for the correct place on earth
    nas.setReferenceValues(ReferenceConfig::defaultReferenceValues);
}

bool NASController::start()
{
    // Add the task to the scheduler
    size_t result = scheduler->addTask(bind(&NASController::update, this),
                                       NASConfig::UPDATE_PERIOD,
                                       TaskScheduler::Policy::RECOVER);

    return ActiveObject::start() && result != 0;
}

void NASController::update()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Update the NAS state only if the FSM is active
    if (this->testState(&NASController::state_active))
    {
        // Get the IMU data
        BMX160WithCorrectionData imuData =
            modules.get<Sensors>()->getBMX160WithCorrectionLastSample();
        UBXGPSData gpsData = modules.get<Sensors>()->getUbxGpsLastSample();

        LPS22DFData barometerData =
            modules.get<Sensors>()->getLPS22LastSample();
        // NAS prediction
        nas.predictGyro(imuData);
        nas.predictAcc(imuData);

        // NAS correction
        nas.correctMag(imuData);
        nas.correctGPS(gpsData);
        nas.correctBaro(barometerData.pressure);
        // Correct with accelerometer if the acceleration is in specs
        Vector3f acceleration  = static_cast<AccelerometerData>(imuData);
        float accelerationNorm = acceleration.norm();
        if (accelerationValid)
        {
            nas.correctAcc(imuData);
        }
        if ((accelerationNorm <
                 (9.8 + (NASConfig::ACCELERATION_THRESHOLD) / 2) &&
             accelerationNorm >
                 (9.8 - (NASConfig::ACCELERATION_THRESHOLD) / 2)))
        {
            if (!accelerationValid)
            {
                accSampleAfterSpike++;
            }
        }
        else
        {
            accelerationValid   = false;
            accSampleAfterSpike = 0;
        }
        if (accSampleAfterSpike > NASConfig::ACCELERATION_THRESHOLD_SAMPLE)
        {
            accSampleAfterSpike = 0;
            accelerationValid   = true;
        }

        Logger::getInstance().log(nas.getState());
    }
}

void NASController::calibrate()
{
    ModuleManager& modules = ModuleManager::getInstance();

    Vector3f acceleration  = Vector3f::Zero();
    Vector3f magneticField = Vector3f::Zero();
    Stats pressure;

    for (int i = 0; i < NASConfig::CALIBRATION_SAMPLES_COUNT; i++)
    {
        // IMU
        BMX160WithCorrectionData imuData =
            modules.get<Sensors>()->getBMX160WithCorrectionLastSample();
        acceleration += Vector3f(imuData.accelerationX, imuData.accelerationY,
                                 imuData.accelerationZ);

        magneticField +=
            Vector3f(imuData.magneticFieldX, imuData.magneticFieldY,
                     imuData.magneticFieldZ);

        // Static pressure barometer
        LPS22DFData barometerData =
            modules.get<Sensors>()->getLPS22LastSample();
        pressure.add(barometerData.pressure);

        miosix::Thread::sleep(NASConfig::CALIBRATION_SLEEP_TIME);
    }

    // Normalize the data
    acceleration /= NASConfig::CALIBRATION_SAMPLES_COUNT;
    magneticField /= NASConfig::CALIBRATION_SAMPLES_COUNT;
    acceleration.normalize();
    magneticField.normalize();

    // Use the triad algorithm to estimate the initial orientation
    StateInitializer state;
    state.triad(acceleration, magneticField, ReferenceConfig::nedMag);

    // Set the pressure reference using an already existing reference values
    ReferenceValues reference = nas.getReferenceValues();
    reference.refPressure     = pressure.getStats().mean;

    // Set reference altitude using barometric measure
    reference.refAltitude = Aeroutils::relAltitude(pressure.getStats().mean);

    // If in this moment the GPS has fix i use that position as starting
    UBXGPSData gps = modules.get<Sensors>()->getUbxGpsLastSample();
    if (gps.fix != 0)
    {
        // We don't set the altitude with the GPS because of not precise
        // measurements
        reference.refLatitude  = gps.latitude;
        reference.refLongitude = gps.longitude;
    }

    // Update the algorithm reference values
    {
        miosix::PauseKernelLock lock;
        nas.setX(state.getInitX());
        nas.setReferenceValues(reference);
    }

    EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
}

void NASController::setCoordinates(Vector2f position)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = nas.getReferenceValues();
    reference.refLatitude     = position[0];
    reference.refLongitude    = position[1];
    nas.setReferenceValues(reference);
}

void NASController::setOrientation(float yaw, float pitch, float roll)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    Matrix<float, 13, 1> x = nas.getX();

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
    reference.refAltitude     = altitude;
    nas.setReferenceValues(reference);
}

void NASController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = nas.getReferenceValues();

    // The temperature is in degrees, converted in kelvin
    reference.refTemperature = temperature + 273.15f;
    nas.setReferenceValues(reference);
}

void NASController::setReferenceValues(const ReferenceValues reference)
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    nas.setReferenceValues(reference);
}

NASControllerStatus NASController::getStatus()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return status;
}

NASState NASController::getNasState()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
    return nas.getState();
}

ReferenceValues NASController::getReferenceValues()
{
    // Need to pause the kernel
    miosix::PauseKernelLock l;
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
            // Calibrate the NAS
            calibrate();
            return logStatus(NASControllerState::CALIBRATING);
        }
        case NAS_READY:
        {
            return transition(&NASController::state_ready);
        }
    }
}

// State skipped on entry because we don't have state_armed and state_disarmed
// in the FMM
void NASController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(NASControllerState::READY);
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
        case NAS_CALIBRATE:
        {
            return transition(&NASController::state_calibrating);
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
    // Update the current FSM state
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    // Log the status
    Logger::getInstance().log(status);
}
}  // namespace Parafoil
