/* Copyright (c) 2024-2026 Skyward Experimental Rocketry
 * Authors: Davide Mor, Pietro Bortolus
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
#include <Main/Configs/SensorsConfig.h>
#include <Main/Sensors/SensorsData.h>
#include <Main/StatsRecorder/StatsRecorder.h>
#include <common/MedianFilter.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/adc/InternalADC.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/AS5047D/AS5047DSPI.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/ND015X/ND015A.h>
#include <sensors/RotatedIMU/RotatedIMU.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/Vectornav/VN100/VN100Spi.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <sensors/correction/TwelveParametersCorrector/TwelveParametersCorrector.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>
#include <mutex>
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

    void resetMagCalibrator();
    void enableMagCalibrator();
    void disableMagCalibrator();
    bool saveMagCalibration();

    Boardcore::AS5047DData getAS5047DLeftLastSample();
    Boardcore::AS5047DData getAS5047DRightLastSample();
    Boardcore::LIS2MDLData getLIS2MDLRcsLastSample();
    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::VN100SpiData getVN100LastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::LIS2MDLData getLIS2MDLIntLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRXLowLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRXHighLastSample();
    Boardcore::ND015XData getND015A0LastSample();
    Boardcore::ND015XData getND015A1LastSample();
    Boardcore::ND015XData getND015A2LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();

    Boardcore::LIS2MDLData getCalibratedLIS2MDLRcsLastSample();
    Boardcore::LIS2MDLData getCalibratedLIS2MDLIntLastSample();
    Boardcore::LSM6DSRXData getCalibratedLSM6DSRXLowLastSample();
    Boardcore::LSM6DSRXData getCalibratedLSM6DSRXHighLastSample();
    Boardcore::VN100SpiData getCalibratedVN100LastSample();
    Boardcore::IMUData getIMULastSample();

    Boardcore::VoltageData getBatteryVoltageLastSample();
    Boardcore::VoltageData getCamBatteryVoltageLastSample();

    Boardcore::PressureData getAtmosPressureLastSample();
    Boardcore::TemperatureData getTemperatureLastSample();

    Boardcore::PressureData getCanPitotTotalPressure();
    Boardcore::PressureData getCanPitotStaticPressure();
    Boardcore::PressureData getCanPitotDynamicPressure();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

    /**
     * Sets the ascent phase for double LSM6DSRX sensor management
     * @param isAscent True if the rocket is in ascent phase, false otherwise.
     */
    void setAscentPhase(bool isAscent);

    // Methods for CanHandler
    void setCanPitotTotalPressure(Boardcore::PressureData data);
    void setCanPitotStaticPressure(Boardcore::PressureData data);

protected:
    virtual bool postSensorCreationHook() { return true; }

    virtual void lsm6dsrxLowCallback();
    virtual void lsm6dsrxHighCallback();
    virtual void vn100Callback();

    Boardcore::TaskScheduler& getSensorsScheduler();

    std::mutex canMutex;
    // Payload
    Boardcore::PressureData canPitotTotalPressure;
    Boardcore::PressureData canPitotStaticPressure;

    // Digital sensors
    std::unique_ptr<Boardcore::AS5047DSPI> as5047d_left;
    std::unique_ptr<Boardcore::AS5047DSPI> as5047d_right;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl_rcs;
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::VN100Spi> vn100;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl_int;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx_low;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx_high;
    std::unique_ptr<Boardcore::ND015A> nd015a_0;
    std::unique_ptr<Boardcore::ND015A> nd015a_1;
    std::unique_ptr<Boardcore::ND015A> nd015a_2;

    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    // Virtual sensors
    std::unique_ptr<Boardcore::RotatedIMU> rotatedImu;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    void as5047dLeftInit();
    void as5047dLeftCallback();

    void as5047dRightInit();
    void as5047dRightCallback();

    void lis2mdlRcsInit();
    void lis2mdlRcsCallback();

    void lps22dfInit();
    void lps22dfCallback();

    void h3lis331dlInit();
    void h3lis331dlCallback();

    void ubxgpsInit();
    void ubxgpsCallback();

    void lis2mdlIntInit();
    void lis2mdlIntCallback();

    void lsm6dsrxLowInit();

    void lsm6dsrxHighInit();

    void vn100Init();

    void internalAdcInit();
    void internalAdcCallback();

    void nd015a0Init();
    void nd015a0Callback();

    void nd015a1Init();
    void nd015a1Callback();

    void nd015a2Init();
    void nd015a2Callback();

    void rotatedImuInit();
    void rotatedImuCallback();

    bool sensorManagerInit();

    bool ascentPhase = true;

    std::mutex magCalibrationMutex;
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    uint8_t magCalibrationTaskId = 0;

    std::mutex lsm6CalibrationLowMutex;
    Boardcore::TwelveParametersCorrector accCalibrationLow;
    Boardcore::TwelveParametersCorrector gyroCalibrationLow;

    std::mutex lsm6CalibrationHighMutex;
    Boardcore::TwelveParametersCorrector accCalibrationHigh;
    Boardcore::TwelveParametersCorrector gyroCalibrationHigh;

    std::mutex vn100CalibrationMutex;

    Boardcore::TwelveParametersCorrector accVN100Calibration;
    Boardcore::TwelveParametersCorrector gyroVN100Calibration;
    Boardcore::SixParametersCorrector magVN100Calibration;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    Common::MedianFilter<float, 3> atmosPressureFilter;

    std::atomic<bool> started{false};
};

}  // namespace Main
