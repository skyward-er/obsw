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

#include <Payload/Main/SensorsStatus.h>
#include <drivers/adc/InternalADC.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>
#include <sensors/analog/pressure/nxp/MPXHZ6130A.h>

namespace Payload
{
class Sensors
{
public:
    // All the sensors that need to be sampled
    Boardcore::InternalADC* internalAdc;
    Boardcore::BatteryVoltageSensor* batteryVoltage;
    Boardcore::MS5803* digitalPressure;
    Boardcore::ADS1118* adcADS1118;

    // TODO need to specify these sensors below
    Boardcore::SSCDANN030PAA* dplVanePressure;
    Boardcore::MPXHZ6130A* staticPortPressure;
    Boardcore::SSCDRRN015PDA* pitotPressure;

    Boardcore::BMX160* imuBMX160;
    Boardcore::BMX160WithCorrection* correctedImuBMX160;
    Boardcore::LIS3MDL* magnetometerLIS3MDL;
    Boardcore::UBXGPSSerial* gpsUblox;

    // The sensor manager object. It sets all the tasks inside the sensors
    // scheduler
    Boardcore::SensorManager* sensorManager;

    /**
     * @brief Construct a new Sensors object
     * @param spiBus The spi bus where all the sensors communicate
     * @param scheduler The sensors dedicated task scheduler
     */
    Sensors(Boardcore::SPIBusInterface& spiBus,
            Boardcore::TaskScheduler* scheduler);

    /**
     * @brief Destroy the Sensors object and all its sensors.
     */
    ~Sensors();

    /**
     * @brief Starts all the calibration procedures regarding some of the most
     * critical sensors.
     */
    void calibrate();

    /**
     * @brief Starts the sensor manager and the task scheduler.
     * @return True if all goes correctly.
     */
    bool start();

    bool isStarted();

    // Kernel lock getters
    Boardcore::InternalADCData getInternalAdcLastSample();
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();
    Boardcore::MS5803Data getDigitalPressureLastSample();
    Boardcore::ADS1118Data getAdcADS1118LastSample();
    Boardcore::SSCDANN030PAAData getDplVanePressureLastSample();
    Boardcore::MPXHZ6130AData getStaticPortPressureLastSample();
    Boardcore::SSCDRRN015PDAData getPitotPressureLastSample();
    Boardcore::BMX160Data getImuBMX160LastSample();
    Boardcore::BMX160WithCorrectionData getCorrectedImuBMX160LastSample();
    Boardcore::LIS3MDLData getMagnetometerLIS3MDLLastSample();
    Boardcore::UBXGPSData getGPSLastSample();

private:
    // The sensors SPI bus
    Boardcore::SPIBusInterface& spiBus;

    // The sensor mapping to be passed to the sensor manager that has to know
    // what sensor it should sample
    Boardcore::SensorManager::SensorMap_t sensorsMap;

    // Debug serial logger
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    // SD logger
    Boardcore::Logger* SDlogger;

    // After-Init sensors status to be logged
    SensorsStatus status;

    // All the sensors sampling callbacks and init functions
    void internalAdcInit();
    void internalAdcCallback();

    void batteryVoltageInit();
    void batteryVoltageCallback();

    void digitalPressureInit();
    void digitalPressureCallback();

    void adcADS1118Init();
    void adcADS1118Callback();

    void dplVanePressureInit();
    void dplVanePressureCallback();

    void staticPortPressureInit();
    void staticPortPressureCallback();

    void pitotPressureInit();
    void pitotPressureCallback();

    void imuBMX160Init();
    void imuBMX160Callback();

    void correctedImuBMX160Init();
    void correctedImuBMX160Callback();

    void magnetometerLIS3MDLInit();
    void magnetometerLIS3MDLCallback();

    void gpsUbloxInit();
    void gpsUbloxCallback();

    /**
     * @brief Method to update the sensors status struct.
     * It needs to be called AFTER the sensorManager start
     */
    void updateSensorsStatus();
};
}  // namespace Payload
