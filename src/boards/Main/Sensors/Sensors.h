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
#include <algorithms/ReferenceValues.h>
#include <common/ReferenceConfig.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/adc/InternalADC.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>
#include <vector>

#include "RotatedIMU/RotatedIMU.h"

namespace Main
{

struct SensorsCalibrationParameter
{
    uint64_t timestamp;
    float referencePressure;
    float offsetStatic1;
    float offsetStatic2;
    float offsetDeployment;

    SensorsCalibrationParameter(uint64_t timestamp, float referencePressure,
                                float offsetStatic1, float offsetStatic2,
                                float offsetDeployment)
        : timestamp(timestamp), referencePressure(referencePressure),
          offsetStatic1(offsetStatic1), offsetStatic2(offsetStatic2),
          offsetDeployment(offsetDeployment)
    {
    }

    SensorsCalibrationParameter() : SensorsCalibrationParameter(0, 0, 0, 0, 0)
    {
    }

    static std::string header()
    {
        return "timestamp,referencePressure,offsetStatic1,offsetStatic2,"
               "offsetDeployment\n";
    }

    void print(std::ostream &os) const
    {
        os << timestamp << "," << referencePressure << "," << offsetStatic1
           << "," << offsetStatic2 << "," << offsetDeployment << "\n";
    }
};

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler>
{
public:
    Sensors() : reference{Common::ReferenceConfig::defaultReferenceValues} {}

    bool isStarted();

    [[nodiscard]] bool start();

    void calibrate();

    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::LPS28DFWData getLPS28DFWLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample();
    Boardcore::ADS131M08Data getADS131M08LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();
    RotatedIMUData getRotatedIMULastSample();
    Boardcore::MagnetometerData getCalibratedMagnetometerLastSample();

    Boardcore::VoltageData getBatteryVoltage();
    Boardcore::VoltageData getCamBatteryVoltage();

    Boardcore::PressureData getStaticPressure1();
    Boardcore::PressureData getStaticPressure2();
    Boardcore::PressureData getDplBayPressure();

    Boardcore::PressureData getCanTopTankPress1();
    Boardcore::PressureData getCanTopTankPress2();
    Boardcore::PressureData getCanCCPress();
    Boardcore::TemperatureData getCanTankTemp();
    Boardcore::VoltageData getCanMotorBatteryVoltage();

    Boardcore::ReferenceValues getReferenceValues()
    {
        miosix::Lock<miosix::FastMutex> l(referenceMutex);
        return reference;
    }

    void setRefAtmosphere(float refTemperature, float refPressure)
    {
        float mslTemperature = Boardcore::Aeroutils::relTemperature(
            -reference.refAltitude, refTemperature);
        float mslPressure = Boardcore::Aeroutils::relPressure(
            -reference.refAltitude, refPressure, refTemperature);

        miosix::Lock<miosix::FastMutex> l(referenceMutex);
        reference.refTemperature = refTemperature;
        reference.refPressure    = refPressure;
        reference.mslTemperature = mslTemperature;
        reference.mslPressure    = mslPressure;
    }

    bool writeMagCalibration();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

    // Methods for CanHandler
    void setCanTopTankPress1(Boardcore::PressureData data);
    void setCanTopTankPress2(Boardcore::PressureData data);
    void setCanCCPress(Boardcore::PressureData data);
    void setCanTankTemp(Boardcore::TemperatureData data);
    void setCanMotorBatteryVoltage(Boardcore::VoltageData data);

protected:
    virtual bool postSensorCreationHook() { return true; }

    miosix::FastMutex canMutex;
    Boardcore::PressureData canCCPressure;
    Boardcore::PressureData canTopTankPressure1;
    Boardcore::PressureData canTopTankPressure2;
    Boardcore::TemperatureData canTankTemperature;
    Boardcore::VoltageData canMotorBatteryVoltage;

    // Digital sensors
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::LPS28DFW> lps28dfw;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Main::RotatedIMU> imu;

    // Analog sensors
    std::unique_ptr<Boardcore::MPXH6115A> staticPressure1;
    std::unique_ptr<Boardcore::MPXH6115A> staticPressure2;
    std::unique_ptr<Boardcore::MPXH6115A> dplBayPressure;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    void lps22dfInit();
    void lps22dfCallback();

    void lps28dfwInit();
    void lps28dfwCallback();

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

    void internalAdcInit();
    void internalAdcCallback();

    void staticPressure1Init();
    void staticPressure1Callback();

    void rotatedIMUInit();
    void rotatedIMUCallback();

    void staticPressure2Init();
    void staticPressure2Callback();

    void dplBayPressureInit();
    void dplBayPressureCallback();

    miosix::FastMutex referenceMutex;
    Boardcore::ReferenceValues reference;

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    bool sensorManagerInit();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};
};

}  // namespace Main