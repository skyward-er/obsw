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
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <algorithms/NAS/StateInitializer.h>
#include <common/ReferenceConfig.h>
#include <common/events/Events.h>
#include <utils/AeroUtils/AeroUtils.h>

using namespace std;
using namespace Eigen;
using namespace Boardcore;
using namespace Main::NASConfig;
using namespace Common;
using namespace Common::ReferenceConfig;

namespace Main
{

bool NASController::start()
{
    // Add the update task to the scheduler
    BoardScheduler::getInstance().getScheduler().addTask(
        bind(&NASController::update, this), UPDATE_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return ActiveObject::start();
}

void NASController::update()
{
    // If the nas is not active i skip the step
    if (this->testState(&NASController::state_active))
    {
        auto imuData =
            Sensors::getInstance().getBMX160WithCorrectionLastSample();
        auto gpsData = Sensors::getInstance().getUbxGpsLastSample();
        auto pressureData =
            Sensors::getInstance().getStaticPressureLastSample();

        // Predict step
        nas.predictGyro(imuData);
        nas.predictAcc(imuData);

        // Correct step
        nas.correctMag(imuData);
        nas.correctGPS(gpsData);
        nas.correctBaro(pressureData.pressure);

        // Correct with accelerometer if the acceleration is in specs
        Vector3f acceleration = static_cast<AccelerometerData>(imuData);
        if (acceleration.norm() > (9.8 + ACCELERATION_THRESHOLD) ||
            acceleration.norm() < (9.8 - ACCELERATION_THRESHOLD))
            accelerationValid = false;
        if (accelerationValid)
            nas.correctAcc(imuData);

        Logger::getInstance().log(nas.getState());
        FlightStatsRecorder::getInstance().update(nas.getState());
    }

#ifdef HILSimulation
    updateData(nas.getState());
#endif  // HILSimulation
}

void NASController::calibrate()
{
    Vector3f acceleration  = Vector3f::Zero();
    Vector3f magneticField = Vector3f::Zero();
    Stats pressure;

    for (int i = 0; i < CALIBRATION_SAMPLES_COUNT; i++)
    {
        // IMU
        BMX160WithCorrectionData imuData =
            Sensors::getInstance().getBMX160WithCorrectionLastSample();
        acceleration += Vector3f(imuData.accelerationX, imuData.accelerationY,
                                 imuData.accelerationZ);
        magneticField +=
            Vector3f(imuData.magneticFieldX, imuData.magneticFieldY,
                     imuData.magneticFieldZ);

        // Barometer
        auto barometerData =
            Sensors::getInstance().getStaticPressureLastSample();
        pressure.add(barometerData.pressure);

        miosix::Thread::sleep(CALIBRATION_SLEEP_TIME);
    }

    // Normalize the data
    acceleration /= CALIBRATION_SAMPLES_COUNT;
    magneticField /= CALIBRATION_SAMPLES_COUNT;
    acceleration.normalize();
    magneticField.normalize();

    // Use the triad algorithm to estimate the initial orientation
    StateInitializer state;
    state.triad(acceleration, magneticField, nedMag);

    // Set the pressure reference using an already existing reference values
    ReferenceValues reference = nas.getReferenceValues();
    reference.refPressure     = pressure.getStats().mean;
    reference.refAltitude     = Aeroutils::relAltitude(
            reference.refPressure, reference.mslPressure, reference.mslTemperature);

    // If in this moment the GPS has fix i use that position as starting
    UBXGPSData gps = Sensors::getInstance().getUbxGpsLastSample();
    if (gps.fix != 0)
    {
        reference.refLatitude  = gps.latitude;
        reference.refLongitude = gps.longitude;
    }

    // Update the algorithm reference values
    {
        miosix::PauseKernelLock l;
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
    reference.refAltitude     = altitude;
    nas.setReferenceValues(reference);
}

void NASController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock l;

    ReferenceValues reference = nas.getReferenceValues();
    reference.refTemperature  = temperature;
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
            logStatus(NASControllerState::CALIBRATING);

            // Calibrate the NAS
            calibrate();

            return;
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
        case NAS_FORCE_START:
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
        case NAS_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            return transition(&NASController::state_ready);
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
    : FSM(&NASController::state_idle), nas(config)
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
    nas.setReferenceValues(defaultReferenceValues);
}

NASController::~NASController()
{
    EventBroker::getInstance().unsubscribe(this);
}

}  // namespace Main
