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

#include <Payload/Buses.h>
#include <Payload/Configs/SensorsConfig.h>
#include <Payload/Sensors/RotatedIMU/RotatedIMU.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/SensorData.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/Pitot/Pitot.h>
#include <sensors/analog/Pitot/PitotData.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN015PA.h>
#include <sensors/analog/pressure/honeywell/SSCMRNN030PA.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "SensorData.h"

namespace Payload
{

class Sensors : public Boardcore::Module
{
public:
    explicit Sensors(Boardcore::TaskScheduler* sched, Buses* buses);

    [[nodiscard]] virtual bool start();

    /**
     * @brief Stops the sensor manager
     * @warning Stops the passed scheduler
     */
    virtual void stop();

    /**
     * @brief Returns if all the sensors are started successfully
     */
    virtual bool isStarted();

    /**
     * @brief Calibrates the sensors with an offset
     */
    virtual void calibrate();

    /**
     * @brief Takes the result of the live magnetometer calibration and applies
     * it to the current calibration + writes it in the csv file
     *
     * @return true if the write was successful
     */
    virtual bool writeMagCalibration();

    // Sensor getters
    virtual Boardcore::LPS22DFData getLPS22DFLastSample();
    virtual Boardcore::LPS28DFWData getLPS28DFW_1LastSample();
    virtual Boardcore::LPS28DFWData getLPS28DFW_2LastSample();
    virtual Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    virtual Boardcore::LIS2MDLData getLIS2MDLLastSample();
    virtual Boardcore::UBXGPSData getGPSLastSample();
    virtual Boardcore::LSM6DSRXData getLSM6DSRXLastSample();
    virtual Boardcore::ADS131M08Data getADS131M08LastSample();

    // Processed getters
    virtual Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();
    virtual Boardcore::BatteryVoltageSensorData
    getCamBatteryVoltageLastSample();
    virtual Boardcore::CurrentData getCurrentLastSample();
    virtual RotatedIMUData getIMULastSample();
    virtual Boardcore::MagnetometerData getCalibratedMagnetometerLastSample();
    virtual Boardcore::HSCMRNN015PAData getStaticPressureLastSample();
    virtual Boardcore::SSCMRNN030PAData getDynamicPressureLastSample();
    virtual Boardcore::PitotData getPitotLastSample();

    void pitotSetReferenceAltitude(float altitude);
    void pitotSetReferenceTemperature(float temperature);

    std::array<Boardcore::SensorInfo, 8> getSensorInfo();

protected:
    /**
     * @brief Method to put a sensor in the sensorMap with the relative infos
     */
    template <typename T>
    void registerSensor(Boardcore::Sensor<T>* sensor, const std::string& name,
                        uint32_t period, std::function<void(void)> callback)
    {
        // Emplace the sensor inside the map
        Boardcore::SensorInfo info(name, period, callback);
        sensorMap.emplace(std::make_pair(sensor, info));
    }

    /**
     * @brief Insert a sensor in the infoGetter.
     */
    template <typename T>
    void addInfoGetter(Boardcore::Sensor<T>* sensor)
    {
        // Add the sensor info getter to the array
        sensorsInit[sensorsId++] = [&]() -> Boardcore::SensorInfo
        { return manager->getSensorInfo(lps22df); };
    }

    // Init and callbacks methods
    void lps22dfInit();
    virtual void lps22dfCallback();

    void lps28dfw_1Init();
    virtual void lps28dfw_1Callback();

    void lps28dfw_2Init();
    virtual void lps28dfw_2Callback();

    void h3lis331dlInit();
    virtual void h3lis331dlCallback();

    void lis2mdlInit();
    virtual void lis2mdlCallback();

    void ubxgpsInit();
    virtual void ubxgpsCallback();

    void lsm6dsrxInit();
    virtual void lsm6dsrxCallback();

    void ads131m08Init();
    virtual void ads131m08Callback();

    void staticPressureInit();
    virtual void staticPressureCallback();

    void dynamicPressureInit();
    virtual void dynamicPressureCallback();

    void pitotInit();
    virtual void pitotCallback();

    void imuInit();
    virtual void imuCallback();

    // Fake processed sensors
    RotatedIMU* imu = nullptr;

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;
    Payload::Buses* buses               = nullptr;
    uint8_t sensorsId;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");

    std::array<std::function<Boardcore::SensorInfo()>,
               SensorsConfig::NUMBER_OF_SENSORS>
        sensorsInit;

    // Sensors instances
    Boardcore::LPS22DF* lps22df       = nullptr;
    Boardcore::LPS28DFW* lps28dfw_1   = nullptr;
    Boardcore::LPS28DFW* lps28dfw_2   = nullptr;
    Boardcore::H3LIS331DL* h3lis331dl = nullptr;
    Boardcore::LIS2MDL* lis2mdl       = nullptr;
    Boardcore::UBXGPSSpi* ubxgps      = nullptr;
    Boardcore::LSM6DSRX* lsm6dsrx     = nullptr;
    Boardcore::ADS131M08* ads131m08   = nullptr;

    // Fake processed sensors
    Boardcore::HSCMRNN015PA* staticPressure  = nullptr;
    Boardcore::SSCMRNN030PA* dynamicPressure = nullptr;
    Boardcore::Pitot* pitot                  = nullptr;
};
}  // namespace Payload
