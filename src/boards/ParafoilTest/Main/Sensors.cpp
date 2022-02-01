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

#include <Configs/SensorsConfig.h>
#include <sensors/SensorInfo.h>
#include "Sensors.h"

using std::bind;
//using std::function;

using namespace Boardcore;

namespace ParafoilTestDev
{
    /**
     * PRIVATE METHODS
     */  
    void Sensors::MPU9250init()
    {
        //I create first a new SPISlave with all the settings
        SPISlave slave{spiInterface, IMU_CS, SPIBusConfig{}};

        //Instantiate the object
        imu_mpu9250 = new MPU9250(slave, 
                                  IMU_SAMPLE_RATE,
                                  IMU_GYRO_SCALE,
                                  IMU_ACCEL_SCALE,
                                  SPI::ClockDivider::DIV_4);

        //Bind the information
        SensorInfo info("MPU9250", IMU_SAMPLE_PERIOD,
                        bind(&Sensors::MPU9250Callback, this), false); 

        //Insert the sensor in the common map
        sensors_map.emplace(std::make_pair(imu_mpu9250, info));

        //Log the results
        LOG_INFO(log, "IMU MPU9250 Setup done!");
    }

    void Sensors::MPU9250Callback()
    {

    }

    void Sensors::UbloxGPSinit()
    {
        //Instantiate the object
        gps_ublox = new UbloxGPSSerial(GPS_BAUD_RATE, GPS_SAMPLE_RATE);

        //Bind the information with the callback method
        SensorInfo info("UbloxGPS", GPS_SAMPLE_PERIOD,
                        bind(&Sensors::UbloxGPSCallback, this), false);

        //Insert the sensor in the common map
        sensors_map.emplace(std::make_pair(gps_ublox, info));

        //Log the result
        LOG_INFO(log, "Ublox GPS Setup done!");
    }

    void Sensors::UbloxGPSCallback()
    {

    }

    void Sensors::BME280init()
    {
        //I first create a SPI slave needed to instantiate the sensor
        SPISlave slave(spiInterface, PRESS_CS, SPIBusConfig{});

        //Instantiate the object
        press_bme280 = new BME280(slave, BME280::BME280_CONFIG_ALL_ENABLED);
        
        //Set the standby time
        press_bme280 -> setStandbyTime(PRESS_SAMPLE_PERIOD);

        //Bind the information with the callback method
        SensorInfo info("BME280", PRESS_SAMPLE_PERIOD,
                        bind(&Sensors::BME280Callback, this), false);

        //Insert the sensor in the common map
        sensors_map.emplace(std::make_pair(press_bme280, info));

        //Log the result
        LOG_INFO(log, "BME280 Setup done!");
    }

    void Sensors::BME280Callback()
    {

    }

    /**
     * PUBLIC METHODS
     */
    Sensors::Sensors(SPIBusInterface& spi, TaskScheduler* scheduler)
        : spiInterface(spi)
    {
        //Sensor init
        MPU9250init();
        UbloxGPSinit();
        BME280init();

        //Sensor manager instance
        //At this point sensors_map contains all the initialized sensors
        manager = new SensorManager(sensors_map, scheduler);
    }

    Sensors::~Sensors()
    {
        //Delete the sensors
        delete(imu_mpu9250);
        delete(gps_ublox);
        delete(press_bme280);

        //Sensor manager stop and delete
        manager -> stop();
        delete manager;
    }

    bool Sensors::start()
    {
        return manager -> start();
    }

    void Sensors::calibrate()
    {

    }
}