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

#include <Payload/Control/NASController.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace Boardcore;
using namespace Eigen;

namespace Payload
{
template <typename IMU, typename GPS>
NASController<IMU, GPS>::NASController(std::function<IMU()> imuGetter,
                                       std::function<GPS()> gpsGetter,
                                       TaskScheduler* scheduler,
                                       NASConfig config)
{
    this->gpsGetter = gpsGetter;
    this->imuGetter = imuGetter;
    this->scheduler = scheduler;
    this->config    = config;

    // Create the nas
    nas = new NAS(config);
}

template <typename IMU, typename GPS>
NASController<IMU, GPS>::~NASController()
{
    delete nas;
}

template <typename IMU, typename GPS>
NASConfig NASController<IMU, GPS>::getDefaultConfig()
{
    NASConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_MAG      = 0.1f;
    config.SIGMA_GPS      = 10.0f;
    config.SIGMA_BAR      = 4.3f;
    config.SIGMA_POS      = 10.0;
    config.SIGMA_VEL      = 10.0;
    config.P_POS          = 1.0f;
    config.P_POS_VERTICAL = 10.0f;
    config.P_VEL          = 1.0f;
    config.P_VEL_VERTICAL = 10.0f;
    config.P_ATT          = 0.01f;
    config.P_BIAS         = 0.01f;
    config.SATS_NUM       = 6.0f;
    // TODO Define what this stuff is
    config.NED_MAG = Vector3f(0.4747, 0.0276, 0.8797);

    return config;
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::setInitialOrientation(Vector3f orientation)
{
    this->initialOrientation = orientation;

    // Set the initial orientation inside the matrix
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat(initialOrientation);
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas->setX(x);
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::setInitialPosition(Vector2f position)
{
    this->initialPosition = position;
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::init()
{
    // Set the reference values (whatever they represent)
    nas->setReferenceValues({0, 0, 0, 110000, 20 + 273.5});

    // Add the update task to the scheduler
    scheduler->addTask(bind(&NASController::step, this), config.T * 1000,
                       TaskScheduler::Policy::RECOVER);
}

template <typename IMU, typename GPS>
bool NASController<IMU, GPS>::start()
{
    // Start the scheduler
    return scheduler->start();
}

template <typename IMU, typename GPS>
NASState NASController<IMU, GPS>::getLastSample()
{
    return nas->getState();
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::step()
{
    // Sample the sensors
    IMU imuData = imuGetter();
    GPS gpsData = gpsGetter();

    // Extrapolate all the data
    Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                          imuData.accelerationZ);
    Vector3f angularVelocity(imuData.angularVelocityX, imuData.angularVelocityY,
                             imuData.angularVelocityZ);
    Vector3f magneticField(imuData.magneticFieldX, imuData.magneticFieldY,
                           imuData.magneticFieldZ);

    // Put the GPS converted data into a 4d vector
    Vector2f gpsPos(gpsData.latitude, gpsData.longitude);
    gpsPos = Aeroutils::geodetic2NED(gpsPos, initialPosition);
    Vector2f gpsVel(gpsData.velocityNorth, gpsData.velocityNorth);
    Vector4f gpsCorrection;
    // cppcheck-suppress constStatement
    gpsCorrection << gpsPos, gpsVel;

    // Calibration
    {
        Vector3f biasAcc(-0.1255, 0.2053, -0.2073);
        acceleration -= biasAcc;
        Vector3f bias(-0.0291, 0.0149, 0.0202);
        angularVelocity -= bias;
        Vector3f offset(15.9850903462129, -15.6775071377074, -33.8438469147423);
        magneticField -= offset;
        magneticField = {magneticField[1], magneticField[0], -magneticField[2]};
    }

    // Predict step
    nas->predictGyro(angularVelocity);
    if (gpsPos[0] < 1e3 && gpsPos[0] > -1e3 && gpsPos[1] < 1e3 &&
        gpsPos[1] > -1e3)
        nas->predictAcc(acceleration);

    // Correct step
    magneticField.normalize();
    nas->correctMag(magneticField);
    acceleration.normalize();
    nas->correctAcc(acceleration);
    if (gpsData.fix)
        nas->correctGPS(gpsCorrection);
    nas->correctBaro(100000);

    auto nasState = nas->getState();
    SDlogger->log(nasState);
}
}  // namespace Payload