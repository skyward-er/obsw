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
#include <drivers/interrupt/external_interrupts.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
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

constexpr uint32_t UPDATE_PERIOD = 20;  // 50 hz

const Eigen::Vector3f nedMag(0.4732, 0.0272, 0.8805);

// static const Boardcore::AxisOrthoOrientation BMX160_AXIS_ROTATION = {
//     Boardcore::Direction::NEGATIVE_Y, Boardcore::Direction::NEGATIVE_Z};
static const Boardcore::AxisOrthoOrientation BMX160_AXIS_ROTATION = {
    Boardcore::Direction::POSITIVE_X, Boardcore::Direction::POSITIVE_Y};

static const Boardcore::ReferenceValues defaultReferenceValues = {
    135.0,              // [m] Altitude
    99714.0,            // [Pa] Pressure
    278.27,             // [K] Temperature
    45.50106793771145,  // [deg] Start latitude
    9.156376900740167,  // [deg] Start longitude
    Boardcore::Constants::MSL_PRESSURE,
    Boardcore::Constants::MSL_TEMPERATURE,
};

static const Boardcore::NASConfig defaultNasConfig = {
    UPDATE_PERIOD / 1000.0,  // T
    0.0001f,                 // SIGMA_BETA
    0.3f,                    // SIGMA_W
    0.1f,                    // SIGMA_ACC
    0.1f,                    // SIGMA_MAG
    10.0f,                   // SIGMA_GPS
    4.3f,                    // SIGMA_BAR
    10.0f,                   // SIGMA_POS
    10.0f,                   // SIGMA_VEL
    10.0f,                   // SIGMA_PITOT
    1.0f,                    // P_POS
    10.0f,                   // P_POS_VERTICAL
    1.0f,                    // P_VEL
    10.0f,                   // P_VEL_VERTICAL
    0.01f,                   // P_ATT
    0.01f,                   // P_BIAS
    6.0f,                    // SATS_NUM
    nedMag                   // NED_MAG
};

void init();
void step();
void print();

BMX160* imu;
BMX160WithCorrection* imuCorrected;
NAS* nas;

SPIBus spi(SPI1);

void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (imu)
        imu->IRQupdateTimestamp(TimestampTimer::getTimestamp());
}

int main()
{
    init();

    TaskScheduler scheduler;
    scheduler.addTask(step, 20, TaskScheduler::Policy::RECOVER);
    scheduler.addTask(print, 250);
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}

void init()
{
    // Prepare bmx160
    enableExternalInterrupt(miosix::sensors::bmx160::intr::getPin().getPort(),
                            miosix::sensors::bmx160::intr::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config bmxConfig;
    bmxConfig.fifoMode              = BMX160Config::FifoMode::HEADER;
    bmxConfig.fifoWatermark         = 20;
    bmxConfig.fifoInterrupt         = BMX160Config::FifoInterruptPin::PIN_INT1;
    bmxConfig.temperatureDivider    = 0;
    bmxConfig.accelerometerRange    = BMX160Config::AccelerometerRange::G_16;
    bmxConfig.gyroscopeRange        = BMX160Config::GyroscopeRange::DEG_2000;
    bmxConfig.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeUnit         = BMX160Config::GyroscopeMeasureUnit::RAD;

    imu = new BMX160(spi, sensors::bmx160::cs::getPin(), bmxConfig, spiConfig);
    imu->init();

    // Prepare calibration
    imuCorrected = new BMX160WithCorrection(imu, BMX160_AXIS_ROTATION);
    imuCorrected->init();

    // NAS
    nas = new NAS(defaultNasConfig);

    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)       = q(0);
    x(7)       = q(1);
    x(8)       = q(2);
    x(9)       = q(3);

    nas->setX(x);
}

void step()
{
    imu->sample();
    imuCorrected->sample();

    auto imuData = imuCorrected->getLastSample();

    // Predict step
    nas->predictGyro({0, 0, 0});
    nas->predictAcc(imuData);

    // Correct step
    nas->correctMag(imuData);
    nas->correctAcc(imuData);
    nas->correctGPS(Vector4f{0, 0, 0, 0});
    nas->correctBaro(defaultReferenceValues.refPressure);
}

void print()
{
    auto nasState = nas->getState();
    auto imuData  = imuCorrected->getLastSample();

    printf("%f, %f, %f\t", imuData.accelerationX, imuData.accelerationY,
           imuData.accelerationZ);
    printf("%f, %f, %f\t", imuData.magneticFieldX, imuData.magneticFieldY,
           imuData.magneticFieldZ);

    printf("w%fwa%fab%fbc%fc\n", nasState.qw, nasState.qx, nasState.qy,
           nasState.qz);
}
