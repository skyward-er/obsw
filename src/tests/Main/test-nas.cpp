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

#include <algorithms/NAS/NAS.h>
#include <algorithms/NAS/StateInitializer.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
void setInitialOrientation();
void init();
void step();
void print();

constexpr uint64_t CALIBRATION_TIMEOUT = 5 * 1e6;

Vector3f nedMag   = Vector3f(0.4747, 0.0276, 0.8797);
Vector2f startPos = Vector2f(45.501141, 9.156281);

NAS* nas;
StateInitializer* stateInitializer;

SPIBus spi2(SPI2);
SPIBus spi4(SPI4);
BMX160* imu  = nullptr;
MS5803* baro = nullptr;

int main()
{
    init();

    nas              = new NAS(getEKConfig());
    stateInitializer = new StateInitializer();
    setInitialOrientation();

    TaskScheduler scheduler;
    scheduler.addTask(step, 20, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(print, 100);
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}

NASConfig getEKConfig()
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
    config.NED_MAG        = nedMag;

    return config;
}

void setInitialOrientation()
{
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas->setX(x);
}

void init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config bmx_config;
    bmx_config.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifoWatermark = 80;
    bmx_config.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmx_config.temperatureDivider = 1;

    bmx_config.accelerometerRange = BMX160Config::AccelerometerRange::G_16;

    bmx_config.gyroscopeRange = BMX160Config::GyroscopeRange::DEG_1000;

    bmx_config.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;

    bmx_config.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    imu =
        new BMX160(spi4, sensors::bmx160::cs::getPin(), bmx_config, spiConfig);
    imu->init();

    baro = new MS5803(spi2, sensors::ms5803::cs::getPin());
    baro->init();

    Logger::getInstance().start();
}

void step()
{
    static int meanCount    = 0;
    static bool calibrating = true;
    static Vector3f accMean = Vector3f::Zero();
    static Vector3f magMean = Vector3f::Zero();

    imu->sample();
    baro->sample();
    auto imuData  = imu->getLastSample();
    auto baroData = baro->getLastSample();

    Vector3f acceleration(imuData.accelerationX, imuData.accelerationY,
                          imuData.accelerationZ);
    Vector3f angularVelocity(imuData.angularVelocityX, imuData.angularVelocityY,
                             imuData.angularVelocityZ);
    Vector3f magneticField(imuData.magneticFieldX, imuData.magneticFieldY,
                           imuData.magneticFieldZ);

    // Calibration
    {
        Vector3f offset{-0.0082, 0.0036, 0.0131};
        angularVelocity = angularVelocity - offset;
        Vector3f b{9.41150, -6.49408, 19.60433};
        Matrix3f A{{0.66017, 0, 0}, {0, 0.66299, 0}, {0, 0, 2.28477}};
        magneticField = (magneticField + b).transpose() * A;
    }

    // Rotate
    {
        acceleration(0)    = -acceleration(0);
        acceleration(1)    = acceleration(1);
        acceleration(2)    = -acceleration(2);
        angularVelocity(0) = -angularVelocity(0);
        angularVelocity(1) = angularVelocity(1);
        angularVelocity(2) = -angularVelocity(2);
        magneticField(0)   = -magneticField(0);
        magneticField(1)   = magneticField(1);
        magneticField(2)   = -magneticField(2);
    }

    if (calibrating)
    {
        if (TimestampTimer::getTimestamp() < CALIBRATION_TIMEOUT)
        {
            accMean = (accMean * meanCount + acceleration) / (meanCount + 1);
            magMean = (magMean * meanCount + acceleration) / (meanCount + 1);
            meanCount++;
        }
        else
        {
            // Now the calibration has ended, compute and log the nas state
            calibrating = false;

            // Compute the initial nas state
            stateInitializer->triad(accMean, magMean, nedMag);
            nas->setX(stateInitializer->getInitX());

            printf("Triad ended\n");
        }
    }

    // Predict step
    nas->predictGyro(angularVelocity);
    // nas->predictAcc(acceleration);

    // Correct step
    magneticField.normalize();
    nas->correctMag(magneticField);
    acceleration.normalize();
    nas->correctAcc(acceleration);
    nas->correctBaro(100000, Constants::MSL_PRESSURE,
                     Constants::MSL_TEMPERATURE);

    // auto nasState = nas->getState();

    // Logger::getInstance().log(imuData);
    // Logger::getInstance().log(baroData);
    // Logger::getInstance().log(nasState);
}

void print()
{
    // auto baroData = baro->getLastSample();
    auto nasState = nas->getState();
    auto imuData  = imu->getLastSample();

    // printf("%f, %f, %f\n", imuData.angularVelocityX,
    // imuData.angularVelocityY,
    //        imuData.angularVelocityZ);

    // printf("%f, %f, %f, %f, %f, %f\n", nasState.n, nasState.e, nasState.d,
    //        nasState.vn, nasState.ve, baroData.pressure);
    printf("w%fwa%fab%fbc%fc\n", nasState.qw, nasState.qx, nasState.qy,
           nasState.qz);
}
