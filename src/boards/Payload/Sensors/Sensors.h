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
    explicit Sensors(Boardcore::TaskScheduler* sched);

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
    // Init and callbacks methods
    virtual void lps22dfInit();
    virtual void lps22dfCallback();

    virtual void lps28dfw_1Init();
    virtual void lps28dfw_1Callback();

    virtual void lps28dfw_2Init();
    virtual void lps28dfw_2Callback();

    virtual void h3lis331dlInit();
    virtual void h3lis331dlCallback();

    virtual void lis2mdlInit();
    virtual void lis2mdlCallback();

    virtual void ubxgpsInit();
    virtual void ubxgpsCallback();

    virtual void lsm6dsrxInit();
    virtual void lsm6dsrxCallback();

    virtual void ads131m08Init();
    virtual void ads131m08Callback();

    virtual void staticPressureInit();
    virtual void staticPressureCallback();

    virtual void dynamicPressureInit();
    virtual void dynamicPressureCallback();

    virtual void pitotInit();
    virtual void pitotCallback();

    virtual void imuInit();
    virtual void imuCallback();

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
    RotatedIMU* imu                          = nullptr;
    Boardcore::HSCMRNN015PA* staticPressure  = nullptr;
    Boardcore::SSCMRNN030PA* dynamicPressure = nullptr;
    Boardcore::Pitot* pitot                  = nullptr;

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    uint8_t sensorsCounter;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");

    std::array<std::function<Boardcore::SensorInfo()>,
               SensorsConfig::NUMBER_OF_SENSORS>
        sensorsInit;
};
}  // namespace Payload
