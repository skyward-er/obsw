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

#include <Main/Configs/SensorsConfig.h>
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
#include <stdint.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
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
     */
    bool writeMagCalibration();

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
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();
    Boardcore::BatteryVoltageSensorData getCamBatteryVoltageLastSample();
    Boardcore::CurrentData getCurrentLastSample();  // (TODO)
    virtual Boardcore::MPXH6400AData getDeploymentPressureLastSample();
    virtual Boardcore::HSCMRNN015PAData getStaticPressure1LastSample();
    virtual Boardcore::HSCMRNN015PAData getStaticPressure2LastSample();
    virtual RotatedIMUData getIMULastSample();
    virtual Boardcore::MagnetometerData getCalibratedMagnetometerLastSample();

    // CAN fake sensors setters
    void setPitot(Boardcore::PitotData data);
    void setCCPressure(Boardcore::PressureData data);
    void setBottomTankPressure(Boardcore::PressureData data);
    void setTopTankPressure(Boardcore::PressureData data);
    void setTankTemperature(Boardcore::TemperatureData data);
    void setMotorBatteryVoltage(Boardcore::BatteryVoltageSensorData data);
    void setMotorCurrent(Boardcore::CurrentData data);

    // CAN fake sensors getters
    virtual Boardcore::PitotData getPitotLastSample();
    virtual Boardcore::PressureData getCCPressureLastSample();
    virtual Boardcore::PressureData getBottomTankPressureLastSample();
    virtual Boardcore::PressureData getTopTankPressureLastSample();
    virtual Boardcore::TemperatureData getTankTemperatureLastSample();
    Boardcore::BatteryVoltageSensorData getMotorBatteryVoltage();
    Boardcore::CurrentData getMotorCurrent();

    // Returns the sensors statuses
    std::array<Boardcore::SensorInfo, SensorsConfig::NUMBER_OF_SENSORS>
    getSensorInfo();

protected:
    RotatedIMU* imu = nullptr;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

private:
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

    virtual void deploymentPressureInit();
    virtual void deploymentPressureCallback();

    virtual void staticPressure1Init();
    virtual void staticPressure1Callback();

    virtual void staticPressure2Init();
    virtual void staticPressure2Callback();

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

    // Can sensors
    Boardcore::PitotData canPitot{0, 0, 0};
    Boardcore::PressureData canCCPressure{0, 0};
    Boardcore::PressureData canBottomTankPressure{0, 0};
    Boardcore::PressureData canTopTankPressure{0, 0};
    Boardcore::TemperatureData canTankTemperature{0, 0};
    Boardcore::BatteryVoltageSensorData canMotorBatteryVoltage{};
    Boardcore::CurrentData canMotorCurrent{};

    // Fake processed sensors
    Boardcore::MPXH6400A* mpxh6400a         = nullptr;
    Boardcore::HSCMRNN015PA* hscmrnn015pa_1 = nullptr;
    Boardcore::HSCMRNN015PA* hscmrnn015pa_2 = nullptr;

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    // Collection of lambdas to get the sensor init statuses
    std::array<std::function<Boardcore::SensorInfo()>,
               SensorsConfig::NUMBER_OF_SENSORS>
        sensorsInit;
    uint8_t sensorsId = 0;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};
}  // namespace Main