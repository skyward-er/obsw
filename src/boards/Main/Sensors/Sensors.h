/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <Main/Sensors/RotatedIMU/RotatedIMU.h>
#include <Main/Sensors/SensorsData.h>
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
#include <sensors/analog/Pitot/PitotData.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN015PA.h>
#include <sensors/analog/pressure/nxp/MPXH6400A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
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
     */
    void writeMagCalibration();

    // Sensor getters
    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::LPS28DFWData getLPS28DFW_1LastSample();
    Boardcore::LPS28DFWData getLPS28DFW_2LastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::UBXGPSData getGPSLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample();
    Boardcore::ADS131M08Data getADS131M08LastSample();

    // Processed getters
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();
    Boardcore::CurrentData getCurrentLastSample();  // (TODO)
    Boardcore::MPXH6400AData getDeploymentPressureLastSample();
    Boardcore::HSCMRNN015PAData getStaticPressure1LastSample();
    Boardcore::HSCMRNN015PAData getStaticPressure2LastSample();
    RotatedIMUData getIMULastSample();
    Boardcore::MagnetometerData getCalibratedMagnetometerLastSample();

    // CAN fake sensors setters
    void setPitot(Boardcore::PitotData data);
    void setCCPressure(Boardcore::PressureData data);
    void setBottomTankPressure(Boardcore::PressureData data);
    void setTopTankPressure(Boardcore::PressureData data);
    void setTankTemperature(Boardcore::TemperatureData data);

    // CAN fake sensors getters
    Boardcore::PitotData getPitotLastSample();
    Boardcore::PressureData getCCPressureLastSample();
    Boardcore::PressureData getBottomTankPressureLastSample();
    Boardcore::PressureData getTopTankPressureLastSample();
    Boardcore::TemperatureData getTankTemperatureLastSample();

private:
    // Init and callbacks methods
    void lps22dfInit();
    void lps22dfCallback();

    void lps28dfw_1Init();
    void lps28dfw_1Callback();

    void lps28dfw_2Init();
    void lps28dfw_2Callback();

    void h3lis331dlInit();
    void h3lis331dlCallback();

    void lis2mdlInit();
    void lis2mdlCallback();

    void ubxgpsInit();
    void ubxgpsCallback();

    void lsm6dsrxInit();
    void lsm6dsrxCallback();

    void ads131m08Init();
    void ads131m08Callback();

    void deploymentPressureInit();
    void deploymentPressureCallback();

    void staticPressure1Init();
    void staticPressure1Callback();

    void staticPressure2Init();
    void staticPressure2Callback();

    void imuInit();
    void imuCallback();

    // Sensors instances
    Boardcore::LPS22DF* lps22df       = nullptr;
    Boardcore::LPS28DFW* lps28dfw_1   = nullptr;
    Boardcore::LPS28DFW* lps28dfw_2   = nullptr;
    Boardcore::H3LIS331DL* h3lis331dl = nullptr;
    Boardcore::LIS2MDL* lis2mdl       = nullptr;
    Boardcore::UBXGPSSpi* ubxgps      = nullptr;
    Boardcore::LSM6DSRX* lsm6dsrx     = nullptr;
    Boardcore::ADS131M08* ads131m08   = nullptr;

    // Can sensors
    Boardcore::PitotData canPitot;
    Boardcore::PressureData canCCPressure;
    Boardcore::PressureData canBottomTankPressure;
    Boardcore::PressureData canTopTankPressure;
    Boardcore::TemperatureData canTankTemperature;

    // Fake processed sensors
    RotatedIMU* imu                         = nullptr;
    Boardcore::MPXH6400A* mpxh6400a         = nullptr;
    Boardcore::HSCMRNN015PA* hscmrnn015pa_1 = nullptr;
    Boardcore::HSCMRNN015PA* hscmrnn015pa_2 = nullptr;

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};
}  // namespace Main