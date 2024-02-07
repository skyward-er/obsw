/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Federico Mandelli
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

#include <Payload/Configs/SensorsConfig.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS1118/ADS1118.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorData.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSerial.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "SensorData.h"

namespace Payload
{

class Sensors : public Boardcore::Module
{
public:
    explicit Sensors(Boardcore::TaskScheduler* sched);

    [[nodiscard]] bool start();

    /**
     * @brief Stops the sensor manager
     * @warning Stops the passed scheduler
     */
    void stop();

    /**
     * @brief Returns if all the sensors are started successfully
     */
    bool isStarted();

    /**
     * @brief Calibrates the sensors with an offset
     */
    void calibrate();

    /**
     * @brief Takes the result of the live magnetometer calibration and applies
     * it to the current calibration + writes it in the csv file
     *
     * @return true if the write was successful
     */
    bool writeMagCalibration();

    Boardcore::BMX160Data getBMX160LastSample();
    Boardcore::BMX160WithCorrectionData getBMX160WithCorrectionLastSample();
    Boardcore::LIS3MDLData getMagnetometerLIS3MDLLastSample();
    Boardcore::MagnetometerData getCalibratedLIS3MDLLastSample();
    Boardcore::MS5803Data getMS5803LastSample();
    Boardcore::UBXGPSData getUbxGpsLastSample();
    Boardcore::ADS1118Data getADS1118LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();

    std::array<Boardcore::SensorInfo, 8> getSensorInfo();

private:
    // Init and callbacks methods
    void bmx160Init();
    void bmx160Callback();

    void bmx160WithCorrectionInit();
    void bmx160WithCorrectionCallback();

    void lis3mdlInit();
    void lis3mdlCallback();

    void ms5803Init();
    void ms5803Callback();

    void ubxGpsInit();
    void ubxGpsCallback();

    void ads1118Init();
    void ads1118Callback();

    void internalADCInit();
    void internalADCCallback();

    void batteryVoltageInit();
    void batteryVoltageCallback();

    // Sensors instances
    Boardcore::LIS3MDL* lis3mdl         = nullptr;
    Boardcore::MS5803* ms5803           = nullptr;
    Boardcore::BMX160* bmx160           = nullptr;
    Boardcore::UBXGPSSerial* ubxGps     = nullptr;
    Boardcore::ADS1118* ads1118         = nullptr;
    Boardcore::InternalADC* internalADC = nullptr;

    // Fake processed sensors
    Boardcore::BMX160WithCorrection* bmx160WithCorrection = nullptr;
    Boardcore::BatteryVoltageSensor* batteryVoltage       = nullptr;

    // Mutexes for sampling
    miosix::FastMutex lis3mdlMutex;
    miosix::FastMutex ms5803Mutex;
    miosix::FastMutex bmx160Mutex;
    miosix::FastMutex bmx160WithCorrectionMutex;
    miosix::FastMutex ubxGpsMutex;
    miosix::FastMutex ads1118Mutex;
    miosix::FastMutex internalADCMutex;
    miosix::FastMutex batteryVoltageMutex;

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    uint8_t sensorsCounter;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");

    std::array<std::function<Boardcore::SensorInfo()>,
               SensorsConfig::NUMBER_OF_SENSORS>
        sensorsInit;
};
}  // namespace Payload
