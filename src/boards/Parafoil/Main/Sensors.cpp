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
#include <Parafoil/ParafoilTest.h>
#include <Parafoil/TelemetriesTelecommands/TMRepository.h>
#include <Parafoil/Wing/WingConfig.h>
#include <math.h>
#include <sensors/SensorInfo.h>
#include <utils/aero/AeroUtils.h>

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
    gps_ublox = new UBXGPSSerial(921600, GPS_SAMPLE_RATE, 2, "gps", 9600);

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
    UBXGPSData d = gps_ublox->getLastSample();
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
    slave.config.mode         = SPI::Mode::MODE_0;

    auto config                      = BME280::BME280_CONFIG_ALL_ENABLED;
    config.bits.oversamplingPressure = BME280::OVERSAMPLING_4;

#ifdef MOCK_SENSORS
    // Instantiate the mock sensor
    mockPressure = new MockPressureSensor();

    // start the sensor
    mockPressure->signalLiftoff();

    // Bind the information with the callback method
    SensorInfo info("BME280", PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::BME280Callback, this), true);
    // Insert the sensor in the common map
    sensors_map.emplace(std::make_pair(mockPressure, info));
#else
    // Instantiate the object
    press_bme280 = new BME280(slave, config);

    // Set the standby time
    press_bme280->setStandbyTime(PRESS_SAMPLE_RATE);

    // Bind the information with the callback method
    SensorInfo info("BME280", PRESS_SAMPLE_PERIOD,
                    bind(&Sensors::BME280Callback, this), true);
    // Insert the sensor in the common map
    sensors_map.emplace(std::make_pair(press_bme280, info));
#endif

    // Log the result
    LOG_INFO(log, "BME280 Setup done!");
}

void Sensors::BME280Callback()
{
    // Log the sample

#ifdef MOCK_SENSORS
    // Create the d data but with the mock informations
    BME280Data d;
    d.pressure          = mockPressure->getLastSample().pressure;
    d.pressureTimestamp = 25;
#else
    BME280Data d = press_bme280->getLastSample();
    logger->log(press_bme280->getLastSample());
#endif
    LOG_DEBUG(log, "{:.2f} {:.2f} {:.2f}", d.pressure, d.temperature,
              d.humidity);

    // Here i put the logic for wing procedure start and flare
    static bool procedureArm = false;
    static bool flareArm     = false;
    static uint8_t count     = 0;
    static Boardcore::Stats pressureMean{};
    static Boardcore::Stats temperatureMean{};

    // If we reache the needed samples
    if (count == WING_PRESSURE_MEAN_COUNT)
    {
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
        float height = aeroutils::relAltitude(pressureMean.getStats().mean,
                                              WING_CALIBRATION_PRESSURE,
                                              WING_CALIBRATION_TEMPERATURE);

        // Update the radio repository
        d.pressure    = pressureMean.getStats().mean;
        d.temperature = temperatureMean.getStats().mean;
        TMRepository::getInstance().update(d);

        // Decide what the wing should do
        if (height > WING_ALGORITHM_ARM_ALTITUDE && !procedureArm)
        {
            procedureArm = true;
        }

        if (height < WING_ALGORITHM_START_ALTITUE && procedureArm)
        {
            // Disarm the starting command
            procedureArm = false;

            // Arm the flare command
            flareArm = true;

            // Start the algorithm
            ParafoilTest::getInstance().wingController->start();
        }

        if (height < WING_FLARE_ALTITUDE && flareArm)
        {
            // Disarm the starting command
            flareArm = false;

            // Flare
            ParafoilTest::getInstance().wingController->flare();
        }

        // Reset the mean values
        count = 0;
        pressureMean.reset();
        temperatureMean.reset();
    }

    // Add the current value to the mean and increase the counting
    pressureMean.add(d.pressure);
    temperatureMean.add(d.temperature);
    count++;
}

Sensors::Sensors(SPIBusInterface& spi, TaskScheduler* scheduler)
    : spiInterface(spi)
{
    // Take the SD logger singleton
    logger = &Logger::getInstance();

    // Sensor init
    MPU9250init();
    miosix::Thread::sleep(200);
    UbloxGPSinit();
    miosix::Thread::sleep(200);
    BME280init();
    miosix::Thread::sleep(200);

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
#ifdef MOCK_SENSORS
    delete (mockPressure);
#else
    delete (press_bme280);
#endif

    // Sensor manager stop and delete
    manager->stop();
    delete manager;
}

bool Sensors::start() { return manager->start(); }

void Sensors::calibrate()
{
    miosix::PauseKernelLock kLock;
    this->needsCalibration = true;
}

}  // namespace Parafoil
