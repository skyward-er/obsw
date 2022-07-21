/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include "Sensors.h"

#include <Parafoil/Buses.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Wing/WingController.h>
#include <sensors/SensorInfo.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <functional>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Parafoil::SensorsConfig;
using namespace Parafoil::WingConfig;

namespace Parafoil
{

bool Sensors::start() { return sensorManager->start(); }

bool Sensors::isStarted() { return sensorManager->areAllSensorsInitialized(); }

MPU9250Data Sensors::getMpu9250LastSample()
{
    miosix::PauseKernelLock lock;
    return mpu9250->getLastSample();
}

UBXGPSData Sensors::getUbxGpsLastSample()
{
    miosix::PauseKernelLock lock;
    return ubxGps->getLastSample();
}

BME280Data Sensors::getBme280LastSample()
{
    miosix::PauseKernelLock lock;
    return bme280->getLastSample();
}

void Sensors::calibrate()
{
    miosix::PauseKernelLock kLock;
    this->needsCalibration = true;
}

std::map<string, bool> Sensors::getSensorsState()
{
    std::map<string, bool> sensorsState;

    for (auto sensor : sensorsMap)
        sensorsState[sensor.second.id] =
            sensorManager->getSensorInfo(sensor.first).isInitialized;

    return sensorsState;
}

Sensors::Sensors()
{
    // Initialize all the sensors
    mpu9250init();
    miosix::Thread::sleep(200);
    ubxGpsInit();
    miosix::Thread::sleep(200);
    bme280init();
    miosix::Thread::sleep(200);

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);
}

Sensors::~Sensors()
{
    delete mpu9250;
    delete ubxGps;
    delete bme280;

    sensorManager->stop();
    delete sensorManager;
}

void Sensors::mpu9250init()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;
    spiConfig.mode         = SPI::Mode::MODE_3;

    mpu9250 =
        new MPU9250(Buses::getInstance().spi1, sensors::mpu9250::cs::getPin(),
                    spiConfig, IMU_SAMPLE_RATE, IMU_GYRO_SCALE, IMU_ACCEL_SCALE,
                    SPI::ClockDivider::DIV_16);

    SensorInfo info(
        "MPU9250", IMU_SAMPLE_PERIOD,
        [&]() { Logger::getInstance().log(mpu9250->getLastSample()); }, true);

    sensorsMap.emplace(std::make_pair(mpu9250, info));

    LOG_INFO(logger, "IMU MPU9250 Setup done!");
}

void Sensors::ubxGpsInit()
{
    ubxGps = new UBXGPSSerial(256000, GPS_SAMPLE_RATE, 2, "gps", 9600);

    SensorInfo info(
        "UbloxGPS", GPS_SAMPLE_PERIOD,
        [&]() { Logger::getInstance().log(ubxGps->getLastSample()); }, true);

    sensorsMap.emplace(std::make_pair(ubxGps, info));

    LOG_INFO(logger, "Ubx GPS Setup done!");
}

void Sensors::bme280init()
{
    // I first create a SPI slave needed to instantiate the sensor
    SPISlave slave(Buses::getInstance().spi1, sensors::bme280::cs::getPin(),
                   SPIBusConfig{});

    slave.config.clockDivider = SPI::ClockDivider::DIV_16;
    slave.config.mode         = SPI::Mode::MODE_0;

    auto config                      = BME280::BME280_CONFIG_ALL_ENABLED;
    config.bits.oversamplingPressure = BME280::OVERSAMPLING_4;

    // Instantiate the object
    bme280 = new BME280(slave, config);

    // Set the standby time
    bme280->setStandbyTime(PRESS_SAMPLE_RATE);

    // Bind the information with the callback method
    SensorInfo info("BME280", PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::bme280Callback, this), true);
    sensorsMap.emplace(std::make_pair(bme280, info));

    LOG_INFO(logger, "BME280 Setup done!");
}

void Sensors::bme280Callback()
{
    auto data = bme280->getLastSample();
    Logger::getInstance().log(data);

    // Logic for wing procedure start and flare
    static uint8_t count = 0;
    static Boardcore::Stats pressureMean{};
    static Boardcore::Stats temperatureMean{};

    // If we reach the needed samples
    if (count == WING_PRESSURE_MEAN_COUNT)
    {
        static bool procedureArm = false;
        static bool flareArm     = false;

        // In case of calibration needed i calibrate
        if (needsCalibration)
        {
            WING_CALIBRATION_PRESSURE = pressureMean.getStats().mean;
            WING_CALIBRATION_TEMPERATURE =
                temperatureMean.getStats().mean + 273.15;

            // Reset the mean values
            count = 0;
            pressureMean.reset();
            temperatureMean.reset();

            // Set calibrated status
            needsCalibration = false;

            // I don't want to calculate stuff
            return;
        }

        // Calculate the height
        float height = Aeroutils::relAltitude(pressureMean.getStats().mean,
                                              WING_CALIBRATION_PRESSURE,
                                              WING_CALIBRATION_TEMPERATURE);

        // Update the radio repository
        data.pressure    = pressureMean.getStats().mean;
        data.temperature = temperatureMean.getStats().mean;

        // Decide what the wing should do
        if (height > WING_ALGORITHM_ARM_ALTITUDE && !procedureArm)
            procedureArm = true;

        if (height < WING_ALGORITHM_START_ALTITUDE && procedureArm)
        {
            // Disarm the starting command
            procedureArm = false;

            // Arm the flare command
            flareArm = true;

            // Start the algorithm
            WingController::getInstance().start();
        }

        if (height < WING_FLARE_ALTITUDE && flareArm)
        {
            // Disarm the starting command
            flareArm = false;

            // Flare
            WingController::getInstance().flare();
        }

        // Reset the mean values
        count = 0;
        pressureMean.reset();
        temperatureMean.reset();
    }

    // Add the current value to the mean and increase the counting
    pressureMean.add(data.pressure);
    temperatureMean.add(data.temperature);
    count++;
}

}  // namespace Parafoil
