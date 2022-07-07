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
#pragma once

#include <algorithms/NAS/NAS.h>
#include <algorithms/NAS/NASState.h>
#include <algorithms/NAS/StateInitializer.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/Sensor.h>

#include <Eigen/Core>
#include <functional>

/**
 * USAGE:
 * NASController* nas = new NASController(bind()...);
 *
 * nas->setInitialOrientation({});
 * nas->setInitialPosition({});
 * nas->init();
 * nas->start();
 *
 * nas->getLastSample();
 */

namespace Parafoil
{
/**
 * @brief The class expects some parameters that represent the data type
 *
 * @tparam IMU The IMU data type
 * @tparam GPS The GPS data type
 */
template <typename IMU, typename GPS>
class NASController
{
public:
    /**
     * @brief Construct a new NASController object
     *
     * @param imuGetter The imu data getter function (done to avoid sync
     * problems with sensors class)
     * @param gpsGetter The gps data getter function (done to avoid sync
     * problems with sensors class)
     */
    NASController(std::function<IMU()> imuGetter,
                  std::function<GPS()> gpsGetter,
                  Boardcore::TaskScheduler* scheduler,
                  Boardcore::NASConfig config = getDefaultConfig());

    /**
     * @brief Destroy the NASController object
     */
    ~NASController();

    /**
     * @brief Method to calculate using triad the initial orientation
     */
    void calculateInitialOrientation();

    /**
     * @brief Method to set the initial gps position
     */
    void setInitialPosition(Eigen::Vector2f position);

    /**
     * @brief Method to initialize the controller
     */
    void init();

    /**
     * @brief Method to start the nas
     */
    bool start();

    /**
     * @brief Method to get the last sample
     */
    Boardcore::NASState getLastSample();

    /**
     * @brief Method to make the nas go forward with the prediction and
     * correction
     */
    void step();

    /**
     * @brief Default config getter
     */
    static Boardcore::NASConfig getDefaultConfig()
    {
        Boardcore::NASConfig config;

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
        config.NED_MAG = Eigen::Vector3f(0.4747, 0.0276, 0.8797);

        return config;
    }

private:
    // The NAS filter
    Boardcore::NAS* nas;

    // The nas config
    Boardcore::NASConfig config;
    Eigen::Vector3f initialOrientation;
    Eigen::Vector2f initialPosition;

    // Getter functions
    std::function<IMU()> imuGetter;
    std::function<GPS()> gpsGetter;

    // Task scheduler
    Boardcore::TaskScheduler* scheduler;

    // SD Logger
    Boardcore::Logger* SDlogger = &Boardcore::Logger::getInstance();

    // Debug logger
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("NASController");
};

template <typename IMU, typename GPS>
NASController<IMU, GPS>::NASController(std::function<IMU()> imuGetter,
                                       std::function<GPS()> gpsGetter,
                                       Boardcore::TaskScheduler* scheduler,
                                       Boardcore::NASConfig config)
{
    this->gpsGetter = gpsGetter;
    this->imuGetter = imuGetter;
    this->scheduler = scheduler;
    this->config    = config;

    // Create the nas
    nas = new Boardcore::NAS(config);
}

template <typename IMU, typename GPS>
NASController<IMU, GPS>::~NASController()
{
    delete nas;
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::calculateInitialOrientation()
{
    // this->initialOrientation = orientation;

    // Mean 10 accelerometer values
    Eigen::Vector3f accelerometer;
    Eigen::Vector3f magnetometer;
    Eigen::Vector3f nedMag(0.4747, 0.0276, 0.8797);
    Boardcore::StateInitializer state;

    // Mean the values
    for (int i = 0; i < 10; i++)
    {
        IMU measure   = imuGetter();
        accelerometer = accelerometer + Eigen::Vector3f(measure.accelerationX,
                                                        measure.accelerationY,
                                                        measure.accelerationZ);
        magnetometer  = magnetometer + Eigen::Vector3f(measure.magneticFieldX,
                                                       measure.magneticFieldY,
                                                       measure.magneticFieldZ);

        // Wait for some time
        miosix::Thread::sleep(100);
    }
    accelerometer = accelerometer / 10;
    magnetometer  = magnetometer / 10;

    // Normalize the data
    accelerometer.normalize();
    magnetometer.normalize();

    // Triad the initial orientation
    state.triad(accelerometer, magnetometer, nedMag);

    nas->setX(state.getInitX());
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::setInitialPosition(Eigen::Vector2f position)
{
    this->initialPosition = position;
}

template <typename IMU, typename GPS>
void NASController<IMU, GPS>::init()
{
    // Set the reference values (whatever they represent)
    nas->setReferenceValues({0, 0, 0, 110000, 20 + 273.5});

    // Add the update task to the scheduler
    scheduler->addTask(std::bind(&NASController::step, this), config.T * 1000,
                       Boardcore::TaskScheduler::Policy::RECOVER);
}

template <typename IMU, typename GPS>
bool NASController<IMU, GPS>::start()
{
    // Start the scheduler
    return scheduler->start();
}

template <typename IMU, typename GPS>
Boardcore::NASState NASController<IMU, GPS>::getLastSample()
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
    Eigen::Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                                 imuData.accelerationZ);
    Eigen::Vector3f angularVelocity(imuData.angularVelocityX,
                                    imuData.angularVelocityY,
                                    imuData.angularVelocityZ);
    Eigen::Vector3f magneticField(
        imuData.magneticFieldX, imuData.magneticFieldY, imuData.magneticFieldZ);

    // Put the GPS converted data into a 4d vector
    Eigen::Vector2f gpsPos(gpsData.latitude, gpsData.longitude);
    gpsPos = Boardcore::Aeroutils::geodetic2NED(gpsPos, initialPosition);
    Eigen::Vector2f gpsVel(gpsData.velocityNorth, gpsData.velocityNorth);
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
    nas->predictGyro(angularVelocity);
    if (gpsPos[0] < 1e3 && gpsPos[0] > -1e3 && gpsPos[1] < 1e3 &&
        gpsPos[1] > -1e3)
        nas->predictAcc(acceleration);

    // Correct step
    magneticField.normalize();
    nas->correctMag(magneticField);
    // acceleration.normalize();
    // nas->correctAcc(acceleration);
    if (gpsData.fix)
        nas->correctGPS(gpsCorrection);
    nas->correctBaro(100000);

    auto nasState = nas->getState();
    SDlogger->log(nasState);
}
}  // namespace Parafoil