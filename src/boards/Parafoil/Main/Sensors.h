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
#pragma once

#include <Parafoil/mocksensors/MockPressureSensor.h>
#include <sensors/BME280/BME280.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>

/**
 * This class memorizes all the sensors instances.
 * Every instance is initialized by the constructor. Also every
 * sensor has its own status
 */
namespace Parafoil
{

class Sensors
{
private:
    /**
     * @brief Sensor manager object
     */
    Boardcore::SensorManager* manager;

    /**
     * @brief Sensor map that contains every sensor istance
     * that needs to be sampled by the sensor manager
     */
    Boardcore::SensorManager::SensorMap_t sensors_map;

    /**
     * @brief SPI interface passed via constructor
     */
    Boardcore::SPIBusInterface& spiInterface;

    /**
     * @brief Sensors serial logger
     */
    Boardcore::PrintLogger log = Boardcore::Logging::getLogger("sensors");

    /**
     * @brief SD logger singleton
     */
    Boardcore::Logger* logger;

    /**
     * @brief boolean that indicates if the sensors need to calibrate
     */
    bool needsCalibration = false;

    /**
     * @brief The MPU9250 IMU init function and
     * sample callback
     */
    void MPU9250init();
    void MPU9250Callback();

    /**
     * @brief GPS init function and sample callback
     */
    void UbloxGPSinit();
    void UbloxGPSCallback();

    /**
     * @brief Barometer BME280 init function and sample callback
     */
    void BME280init();
    void BME280Callback();

public:
    /**
     * @brief MPU9250IMU
     */
    Boardcore::MPU9250* imu_mpu9250;

    /**
     * @brief GPS
     */
    Boardcore::UBXGPSSerial* gps_ublox;

/**
 * @brief Barometer
 */
#ifdef MOCK_SENSORS
    MockPressureSensor* mockPressure;
#else
    Boardcore::BME280* press_bme280;
#endif

    /**
     * @brief Constructor
     * @param The spi interface used for the sensors
     * @param The task scheduler
     */
    Sensors(Boardcore::SPIBusInterface& spi,
            Boardcore::TaskScheduler* scheduler);

    /**
     * @brief Deconstructor
     */
    ~Sensors();

    /**
     * @brief Starts the sensor manager to sample data
     * @return boolean that indicates operation's result
     */
    bool start();

    /**
     * @brief Calibrates the sensors that need to
     */
    void calibrate();

    /**
     * @brief Lock getters
     */
    Boardcore::MPU9250Data getMPU9250LastSample();
    Boardcore::BME280Data getBME280LastSample();
    Boardcore::UBXGPSData getGPSLastSample();
};
}  // namespace Parafoil
