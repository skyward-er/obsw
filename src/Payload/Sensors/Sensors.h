/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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
#include <Main/Sensors/SensorsData.h>
#include <Payload/Buses.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/ND015X/ND015A.h>
#include <sensors/ND015X/ND015D.h>
#include <sensors/RotatedIMU/RotatedIMU.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/pressure/nxp/MPX5010.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <sensors/correction/TwelveParametersCorrector/TwelveParametersCorrector.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <mutex>

#include "SensorData.h"

namespace Payload
{
class BoardScheduler;
class Buses;
class FlightStatsRecorder;

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler,
                                                     FlightStatsRecorder>
{
public:
    Sensors() {}

    bool isStarted();

    [[nodiscard]] bool start();

    void calibrate();

    Main::CalibrationData getCalibration();

    void resetMagCalibrator();
    void enableMagCalibrator();
    void disableMagCalibrator();
    bool saveMagCalibration();

    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLExtLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRX0LastSample();
    Boardcore::LSM6DSRXData getLSM6DSRX1LastSample();
    Boardcore::ND015XData getND015ADataLastSample();
    Boardcore::ND015XData getND015DDataLastSample();
    Boardcore::InternalADCData getInternalADCLastSample();

    Boardcore::LIS2MDLData getCalibratedLIS2MDLExtLastSample();
    Boardcore::LIS2MDLData getCalibratedLIS2MDLLastSample();
    Boardcore::LSM6DSRXData getCalibratedLSM6DSRX0LastSample();
    Boardcore::LSM6DSRXData getCalibratedLSM6DSRX1LastSample();
    Boardcore::IMUData getIMULastSample();

    StaticPressureData getStaticPressureLastSample();
    DynamicPressureData getDynamicPressureLastSample();

    Boardcore::VoltageData getBatteryVoltageLastSample();
    Boardcore::VoltageData getCamBatteryVoltageLastSample();

    Boardcore::TemperatureData getTemperatureLastSample();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

protected:
    virtual bool postSensorCreationHook() { return true; }

    virtual void lsm6dsrx0Callback();
    virtual void lsm6dsrx1Callback();

    Boardcore::TaskScheduler& getSensorsScheduler();

    // Digital sensors
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl_ext;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx_0;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx_1;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::ND015A> nd015a;
    std::unique_ptr<Boardcore::ND015D> nd015d;

    // Virtual sensors
    std::unique_ptr<Boardcore::RotatedIMU> rotatedImu;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    void lps22dfInit();
    void lps22dfCallback();

    void h3lis331dlInit();
    void h3lis331dlCallback();

    void lis2mdlExtInit();
    void lis2mdlExtCallback();

    void lis2mdlInit();
    void lis2mdlCallback();

    void ubxgpsInit();
    void ubxgpsCallback();

    void lsm6dsrx0Init();

    void lsm6dsrx1Init();

    void internalAdcInit();
    void internalAdcCallback();

    void nd015aInit();
    void nd015aCallback();

    void nd015dInit();
    void nd015dCallback();

    void rotatedImuInit();
    void rotatedImuCallback();

    bool sensorManagerInit();

    std::mutex magCalibrationMutex;
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    uint8_t magCalibrationTaskId = 0;

    std::mutex lsm6Calibration0Mutex;
    Boardcore::TwelveParametersCorrector accCalibration0;
    Boardcore::TwelveParametersCorrector gyroCalibration0;

    std::mutex lsm6Calibration1Mutex;
    Boardcore::TwelveParametersCorrector accCalibration1;
    Boardcore::TwelveParametersCorrector gyroCalibration1;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};
};

}  // namespace Payload
