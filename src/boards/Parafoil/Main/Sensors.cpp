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

#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/TelemetriesTelecommands/TMRepository.h>
#include <sensors/SensorInfo.h>

using std::bind;
using namespace Boardcore;

namespace Parafoil
{

void Sensors::MPU9250init()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;
    spiConfig.mode         = SPI::Mode::MODE_3;
    // I create first a new SPISlave with all the settings
    SPISlave slave{spiInterface, IMU_CS, spiConfig};

    // Instantiate the object
    imu_mpu9250 = new MPU9250(slave, IMU_SAMPLE_RATE, IMU_GYRO_SCALE,
                              IMU_ACCEL_SCALE, SPI::ClockDivider::DIV_16);

    // Bind the information
    SensorInfo info("MPU9250", IMU_SAMPLE_PERIOD,
                    bind(&Sensors::MPU9250Callback, this), true);

    // Insert the sensor in the common map
    sensors_map.emplace(std::make_pair(imu_mpu9250, info));

    // Log the results
    LOG_INFO(log, "IMU MPU9250 Setup done!");
}

void Sensors::MPU9250Callback()
{
    // Log the sample
    MPU9250Data d = imu_mpu9250->getLastSample();
    logger->log(imu_mpu9250->getLastSample());
    LOG_DEBUG(log, "{:.2f} {:.2f} {:.2f}", d.accelerationX, d.accelerationY,
              d.accelerationZ);

    // Update the radio repository
    TMRepository::getInstance().update(d);
}

void Sensors::UbloxGPSinit()
{
    // Instantiate the object TODO set the sample rate and stuff
    gps_ublox = new UbloxGPS(38400, GPS_SAMPLE_RATE, 2, "gps", 38400);

    // Starting GPS thread
    gps_ublox->start();

    // Bind the information with the callback method
    SensorInfo info("UbloxGPS", GPS_SAMPLE_PERIOD,
                    bind(&Sensors::UbloxGPSCallback, this), true);

    // Insert the sensor in the common map
    sensors_map.emplace(std::make_pair(gps_ublox, info));

    // Log the result
    LOG_INFO(log, "Ublox GPS Setup done!");
}

void Sensors::UbloxGPSCallback()
{
    // Log the sample
    UbloxGPSData d = gps_ublox->getLastSample();
    logger->log(gps_ublox->getLastSample());
    if (d.fix)
    {
        LOG_DEBUG(log, "{:.2f} {:.2f}", d.latitude, d.longitude);
    }

    // Update the radio repository
    TMRepository::getInstance().update(d);
}

void Sensors::BME280init()
{
    // I first create a SPI slave needed to instantiate the sensor
    SPISlave slave(spiInterface, PRESS_CS, SPIBusConfig{});

    slave.config.clockDivider = SPI::ClockDivider::DIV_16;

    auto config                      = BME280::BME280_CONFIG_ALL_ENABLED;
    config.bits.oversamplingPressure = BME280::OVERSAMPLING_4;

    // Instantiate the object
    press_bme280 = new BME280(slave, config);

    // Set the standby time
    press_bme280->setStandbyTime(PRESS_SAMPLE_RATE);

    // Bind the information with the callback method
    SensorInfo info("BME280", PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::BME280Callback, this), true);
    // Insert the sensor in the common map
    sensors_map.emplace(std::make_pair(press_bme280, info));

    // Log the result
    LOG_INFO(log, "BME280 Setup done!");
}

void Sensors::BME280Callback()
{
    // Log the sample
    BME280Data d = press_bme280->getLastSample();
    logger->log(press_bme280->getLastSample());
    LOG_DEBUG(log, "{:.2f} {:.2f} {:.2f}", d.pressure, d.temperature,
              d.humidity);

    // Update the radio repository
    TMRepository::getInstance().update(d);
}

Sensors::Sensors(SPIBusInterface& spi, TaskScheduler* scheduler)
    : spiInterface(spi)
{
    // Take the SD logger singleton
    logger = &Logger::getInstance();

    // Sensor init
    MPU9250init();
    UbloxGPSinit();
    BME280init();

    // Sensor manager instance
    // At this point sensors_map contains all the initialized sensors
    // cppcheck-suppress noCopyConstructor
    // cppcheck-suppress noOperatorEq
    manager = new SensorManager(sensors_map, scheduler);
}

Sensors::~Sensors()
{
    // Delete the sensors
    delete (imu_mpu9250);
    delete (gps_ublox);
    delete (press_bme280);

    // Sensor manager stop and delete
    manager->stop();
    delete manager;
}

bool Sensors::start() { return manager->start(); }

void Sensors::calibrate() {}

}  // namespace Parafoil
