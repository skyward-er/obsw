/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Matteo Pignataro
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/NASConfig.h>
#include <Payload/Configs/WingConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <sensors/MPU9250/MPU9250Data.h>
#include <sensors/UBXGPS/UBXGPSData.h>

using namespace std;
using namespace Boardcore;
using namespace Common;

namespace Payload
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

    // Sample the sensors
    BMX160WithCorrectionData imuData =
        Sensors::getInstance().getBMX160WithCorrectionLastSample();
    UBXGPSData gpsData      = Sensors::getInstance().getUbxGpsLastSample();
    MS5803Data pressureData = Sensors::getInstance().getMS5803LastSample();

    // Extrapolate all the data
    Eigen::Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                                 imuData.accelerationZ);
    Eigen::Vector3f angularVelocity(imuData.angularVelocityX,
                                    imuData.angularVelocityY,
                                    imuData.angularVelocityZ);
    Eigen::Vector3f magneticField(
        imuData.magneticFieldX, imuData.magneticFieldY, imuData.magneticFieldZ);

    // // Put the GPS converted data into a 4d vector
    // Eigen::Vector2f gpsPos(gpsData.latitude, gpsData.longitude);
    // gpsPos = Aeroutils::geodetic2NED(gpsPos, initialPosition);
    // Eigen::Vector2f gpsVel(gpsData.velocityNorth, gpsData.velocityEast);
    // Eigen::Vector4f gpsCorrection;
    // // cppcheck-suppress constStatement
    // gpsCorrection << gpsPos, gpsVel;

    // // Calibration
    // {
    //     Eigen::Vector3f biasAcc(-0.1255, 0.2053, -0.2073);
    //     acceleration -= biasAcc;
    //     Eigen::Vector3f bias(-0.0291, 0.0149, 0.0202);
    //     angularVelocity -= bias;
    //     Eigen::Vector3f offset(15.9850903462129, -15.6775071377074,
    //                            -33.8438469147423);
    //     magneticField -= offset;
    //     magneticField = {magneticField[1], magneticField[0],
    //     -magneticField[2]};
    // }

    // Predict step
    nas.predictGyro(angularVelocity);
    // if (gpsPos[0] < 1e3 && gpsPos[0] > -1e3 && gpsPos[1] < 1e3 &&
    //     gpsPos[1] > -1e3)
    // nas.predictAcc(acceleration);

    // Correct step
    magneticField.normalize();
    nas.correctMag(magneticField);
    // if (gpsData.fix)
    //     nas.correctGPS(gpsCorrection);
    nas.correctGPS(gpsData);
    nas.correctBaro(pressureData.pressure);

    NASState nasState = nas.getState();

    Logger::getInstance().log(nasState);
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
    for (int i = 0; i < 10; i++)
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

    accelerometer /= 10;
    magnetometer /= 10;
    pressure /= 10;
    temperature /= 10;

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
    else
    {
        // Change the initial position if not already set
        reference.startLatitude  = reference.startLatitude == 0
                                       ? WingConfig::DEFAULT_GPS_INITIAL_LAT
                                       : reference.startLatitude;
        reference.startLongitude = reference.startLongitude == 0
                                       ? WingConfig::DEFAULT_GPS_INITIAL_LON
                                       : reference.startLongitude;
    }

    // Set the values inside the NAS
    {
        // Need to pause the kernel because the only invocation comes from the
        // radio
        // which is a separate thread
        miosix::PauseKernelLock klock;

        nas.setX(state.getInitX());
        nas.setReferenceValues(reference);
    }

    // At the end i publish on the nas topic the end
    EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
}

void NASController::setInitialPosition(Eigen::Vector2f position)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock klock;

    ReferenceValues reference = nas.getReferenceValues();
    reference.startLatitude   = position[0];
    reference.startLongitude  = position[1];
    nas.setReferenceValues(reference);
}

void NASController::setInitialOrientation(float yaw, float pitch, float roll)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock klock;

    NASState state = nas.getState();

    // TODO talk about efficiency reasons
}

void NASController::setReferenceAltitude(float altitude)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock klock;

    ReferenceValues reference = nas.getReferenceValues();
    reference.altitude        = altitude;
    nas.setReferenceValues(reference);
}

void NASController::setReferenceTemperature(float temperature)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock klock;

    ReferenceValues reference = nas.getReferenceValues();
    reference.temperature     = temperature;
    nas.setReferenceValues(reference);
}

NASState NASController::getNasState()
{
    miosix::PauseKernelLock klock;
    return nas.getState();
}

void NASController::setReferenceValues(const ReferenceValues reference)
{
    // Need to pause the kernel because the only invocation comes from the radio
    // which is a separate thread
    miosix::PauseKernelLock klock;

    nas.setReferenceValues(reference);
}

ReferenceValues NASController::getReferenceValues()
{
    return nas.getReferenceValues();
}

NASController::NASController()
    : nas(NASConfig::config), FSM(&NASController::state_idle)
{
    // Subscribe the FSM to the correct topics
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    EventBroker::getInstance().subscribe(this, TOPIC_FMM);
}

/**
 * FSM Methods
 */

void NASController::state_idle(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            logStatus(NASControllerState::IDLE);
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
            // Calibrate the NAS
            initializeOrientationAndPressure();

            // Log the state
            logStatus(NASControllerState::CALIBRATING);
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
            logStatus(NASControllerState::READY);
            break;
        }
        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }
        case FLIGHT_LIFTOFF:
        {
            transition(&NASController::state_active);
            break;
        }
        case TMTC_FORCE_LAUNCH:
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
            // The nas is automatically activated because the update method
            // looks for the active state to execute the update
            logStatus(NASControllerState::ACTIVE);
            break;
        }
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&NASController::state_end);
            break;
        }
        case TMTC_FORCE_LANDING:
        {
            transition(&NASController::state_end);
            break;
        }
        case FMM_MISSION_TIMEOUT:
        {
            transition(&NASController::state_end);
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
            logStatus(NASControllerState::END);
            break;
        }
    }
}

void NASController::logStatus(NASControllerState state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = state;

    Logger::getInstance().log(status);
}

NASControllerStatus NASController::getStatus() { return status; }

}  // namespace Payload
