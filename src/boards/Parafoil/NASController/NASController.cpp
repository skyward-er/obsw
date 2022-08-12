/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Matteo Pignataro
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/NASConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <sensors/MPU9250/MPU9250Data.h>
#include <sensors/UBXGPS/UBXGPSData.h>

using namespace std;
using namespace Boardcore;

namespace Parafoil
{

void NASController::init()
{
    // Set the reference values (whatever they represent)
    nas.setReferenceValues({0, 0, 0, 110000, 20 + 273.5});
}

bool NASController::start()
{
    // Calculate the initial orientation using triad algorithm
    calculateInitialOrientation();

    // Add the update task to the scheduler
    BoardScheduler::getInstance().getScheduler().addTask(
        bind(&NASController::update, this), NASConfig::UPDATE_PERIOD,
        TaskScheduler::Policy::RECOVER);

    return true;
}

void NASController::update()
{
    // Sample the sensors
    MPU9250Data imuData = Sensors::getInstance().getMpu9250LastSample();
    UBXGPSData gpsData  = Sensors::getInstance().getUbxGpsLastSample();

    // Extrapolate all the data
    Eigen::Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                                 imuData.accelerationZ);
    Eigen::Vector3f angularVelocity(imuData.angularVelocityX,
                                    imuData.angularVelocityY,
                                    imuData.angularVelocityZ);
    Eigen::Vector3f magneticField(
        imuData.magneticFieldX, imuData.magneticFieldY, imuData.magneticFieldZ);

    // Put the GPS converted data into a 4d vector
    Eigen::Vector2f gpsPos(gpsData.latitude, gpsData.longitude);
    gpsPos = Aeroutils::geodetic2NED(gpsPos, initialPosition);
    Eigen::Vector2f gpsVel(gpsData.velocityNorth, gpsData.velocityEast);
    Eigen::Vector4f gpsCorrection;
    // cppcheck-suppress constStatement
    gpsCorrection << gpsPos, gpsVel;

    // Calibration
    {
        Eigen::Vector3f biasAcc(-0.1255, 0.2053, -0.2073);
        acceleration -= biasAcc;
        Eigen::Vector3f bias(-0.0291, 0.0149, 0.0202);
        angularVelocity -= bias;
        Eigen::Vector3f offset(15.9850903462129, -15.6775071377074,
                               -33.8438469147423);
        magneticField -= offset;
        magneticField = {magneticField[1], magneticField[0], -magneticField[2]};
    }

    // Predict step
    nas.predictGyro(angularVelocity);
    if (gpsPos[0] < 1e3 && gpsPos[0] > -1e3 && gpsPos[1] < 1e3 &&
        gpsPos[1] > -1e3)
        nas.predictAcc(acceleration);

    // Correct step
    magneticField.normalize();
    nas.correctMag(magneticField);
    if (gpsData.fix)
        nas.correctGPS(gpsCorrection);
    nas.correctBaro(100000);

    NASState nasState = nas.getState();

    Logger::getInstance().log(nasState);
}

void NASController::calculateInitialOrientation()
{
    // Mean 10 accelerometer values
    Eigen::Vector3f accelerometer;
    Eigen::Vector3f magnetometer;
    StateInitializer state;

    // Mean the values
    for (int i = 0; i < 10; i++)
    {
        MPU9250Data data = Sensors::getInstance().getMpu9250LastSample();
        accelerometer += Eigen::Vector3f(data.accelerationX, data.accelerationY,
                                         data.accelerationZ);
        magnetometer += Eigen::Vector3f(
            data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);

        // Wait for some time
        miosix::Thread::sleep(100);
    }

    accelerometer /= 10;
    magnetometer /= 10;

    // Normalize the data
    accelerometer.normalize();
    magnetometer.normalize();

    // Triad the initial orientation
    state.triad(accelerometer, magnetometer, NASConfig::nedMag);

    nas.setX(state.getInitX());
}

void NASController::setInitialPosition(Eigen::Vector2f position)
{
    initialPosition = position;
}

NASState NASController::getNasState() { return nas.getState(); }

void NASController::setReferenceValues(const ReferenceValues reference)
{
    nas.setReferenceValues(reference);
}

ReferenceValues NASController::getReferenceValues()
{
    return nas.getReferenceValues();
}

NASController::NASController() : nas(NASConfig::config) {}

}  // namespace Parafoil
