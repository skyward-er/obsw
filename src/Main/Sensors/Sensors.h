/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/Sensors/SensorsData.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/adc/InternalADC.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/ND015X/ND015A.h>
#include <sensors/RotatedIMU/RotatedIMU.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/Vectornav/VN100/VN100Spi.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>
#include <vector>

namespace Main
{

class Sensors
    : public Boardcore::InjectableWithDeps<Buses, BoardScheduler, StatsRecorder>
{
public:
    Sensors() {}

    bool isStarted();

    [[nodiscard]] bool start();

    void calibrate();

    CalibrationData getCalibration();

    void setBaroCalibrationReference(float reference);
    void resetBaroCalibrationReference();

    void resetMagCalibrator();
    void enableMagCalibrator();
    void disableMagCalibrator();
    bool saveMagCalibration();

    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::LPS28DFWData getLPS28DFWLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLExtLastSample();
    Boardcore::LIS2MDLData getLIS2MDLInExtLastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRX0LastSample();
    Boardcore::LSM6DSRXData getLSM6DSRX1LastSample();
    Boardcore::VN100SpiData getVN100LastSample();
    Boardcore::ADS131M08Data getADS131M08LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();

    Boardcore::LIS2MDLData getCalibratedLIS2MDLLastSample();
    Boardcore::LSM6DSRXData getCalibratedLSM6DSRXLastSample();

    Boardcore::VoltageData getBatteryVoltageLastSample();
    Boardcore::VoltageData getCamBatteryVoltageLastSample();

    Boardcore::ND015XData getND015A0LastSample();
    Boardcore::ND015XData getND015A1LastSample();
    Boardcore::ND015XData getND015A2LastSample();
    Boardcore::ND015XData getND015A3LastSample();

    Boardcore::IMUData getIMULastSample();
    Boardcore::PressureData getAtmosPressureLastSample();

    Boardcore::PressureData getCanPitotDynamicPressLastSample();
    Boardcore::PressureData getCanPitotStaticPressLastSample();
    Boardcore::PressureData getCanTopTankPressLastSample();
    Boardcore::PressureData getCanBottomTankPressLastSample();
    Boardcore::PressureData getCanCCPressLastSample();
    Boardcore::TemperatureData getCanTankTempLastSample();
    Boardcore::VoltageData getCanMotorBatteryVoltageLastSample();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

    // Methods for CanHandler
    void setCanPitotDynamicPress(Boardcore::PressureData data);
    void setCanPitotStaticPress(Boardcore::PressureData data);
    void setCanTopTankPress(Boardcore::PressureData data);
    void setCanBottomTankPress(Boardcore::PressureData data);
    void setCanCCPress(Boardcore::PressureData data);
    void setCanTankTemp(Boardcore::TemperatureData data);
    void setCanMotorBatteryVoltage(Boardcore::VoltageData data);

protected:
    virtual bool postSensorCreationHook() { return true; }

    virtual void lsm6dsrxCallback();

    Boardcore::TaskScheduler& getSensorsScheduler();

    miosix::FastMutex canMutex;
    Boardcore::PressureData canPitotDynamicPressure;
    Boardcore::PressureData canPitotStaticPressure;
    Boardcore::PressureData canCCPressure;
    Boardcore::PressureData canTopTankPressure;
    Boardcore::PressureData canBottomTankPressure;
    Boardcore::TemperatureData canTankTemperature;
    Boardcore::VoltageData canMotorBatteryVoltage;

    // Digital sensors
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::LPS28DFW> lps28dfw;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl_ext;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl_in_ext;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx_0;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx_1;
    std::unique_ptr<Boardcore::VN100Spi> vn100;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::ND015A> nd015a_0;
    std::unique_ptr<Boardcore::ND015A> nd015a_1;
    std::unique_ptr<Boardcore::ND015A> nd015a_2;
    std::unique_ptr<Boardcore::ND015A> nd015a_3;

    // Virtual sensors
    std::unique_ptr<Boardcore::RotatedIMU> rotatedImu;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    void lps22dfInit();
    void lps22dfCallback();

    void lps28dfwInit();
    void lps28dfwCallback();

    void h3lis331dlInit();
    void h3lis331dlCallback();

    void lis2mdlExtInit();
    void lis2mdlExtCallback();

    void lis2mdlInExtInit();
    void lis2mdlInExtCallback();

    void ubxgpsInit();
    void ubxgpsCallback();

    void lsm6dsrx0Init();
    void lsm6dsrx0Callback();

    void lsm6dsrx1Init();
    void lsm6dsrx1Callback();

    void vn100Init();
    void vn100Callback();

    void ads131m08Init();
    void ads131m08Callback();

    void internalAdcInit();
    void internalAdcCallback();

    void nd015a0Init();
    void nd015a0Callback();

    void nd015a1Init();
    void nd015a1Callback();

    void nd015a2Init();
    void nd015a2Callback();

    void nd015a3Init();
    void nd015a3Callback();

    void rotatedImuInit();
    void rotatedImuCallback();

    bool sensorManagerInit();

    miosix::FastMutex baroCalibrationMutex;
    float baroCalibrationReference   = 0;
    bool useBaroCalibrationReference = false;

    miosix::FastMutex magCalibrationMutex;
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    uint8_t magCalibrationTaskId = 0;

    miosix::FastMutex gyroCalibrationMutex;
    Boardcore::BiasCorrector gyroCalibration;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};
};

}  // namespace Main
