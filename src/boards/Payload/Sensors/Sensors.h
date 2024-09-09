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

#include <Payload/Configs/SensorsConfig.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/RotatedIMU/RotatedIMU.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/pressure/nxp/MPXH6115A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "SensorData.h"

namespace Payload
{
class BoardScheduler;
class Buses;
class FlightStatsRecorder;

/**
 * @brief Manages all the sensors of the payload board.
 */
class Sensors : public Boardcore::InjectableWithDeps<BoardScheduler, Buses,
                                                     FlightStatsRecorder>
{
public:
    [[nodiscard]] bool start();

    /**
     * @brief Returns whether all enabled sensors are started and running.
     */
    bool isStarted();

    /**
     * @brief Calibrates the sensors that need calibration.
     *
     * @note This function is blocking.
     */
    void calibrate();

    /**
     * @brief Returns the current sensors calibration parameters.
     *
     * @internal Ensure mutexes are unlocked before calling this function.
     */
    SensorCalibrationData getCalibrationData();

    /**
     * @brief Resets the magnetometer calibration.
     */
    void resetMagCalibrator();

    /**
     * @brief Enables the magnetometer calibration task.
     */
    void enableMagCalibrator();

    /**
     * @brief Disables the magnetometer calibration task.
     */
    void disableMagCalibrator();

    /**
     * @brief Saves the current magnetometer calibration to file, overwriting
     * the previous one, and sets it as the active calibration.
     *
     * @return Whether the calibration was saved successfully.
     */
    bool saveMagCalibration();

    void setBaroCalibrationReference(float reference);
    void resetBaroCalibrationReference();

    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::LPS28DFWData getLPS28DFWLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample();
    Boardcore::ADS131M08Data getADS131M08LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();

    StaticPressureData getStaticPressureLastSample();
    DynamicPressureData getDynamicPressureLastSample();

    Boardcore::IMUData getIMULastSample();

    Boardcore::BatteryVoltageSensorData getBatteryVoltage();
    Boardcore::BatteryVoltageSensorData getCamBatteryVoltage();
    Boardcore::LIS2MDLData getCalibratedLIS2MDLLastSample();
    Boardcore::LSM6DSRXData getCalibratedLSM6DSRXLastSample();

    /**
     * @brief Returns information about all sensors managed by this class
     */
    std::vector<Boardcore::SensorInfo> getSensorInfo();

protected:
    /**
     * @brief A function that is called after all sensors have been created but
     * before they are inserted into the sensor manager.
     *
     * It can be overridden by subclasses to perform additional setup on the
     * sensors.
     *
     * @return Whether the additional setup was successful. If false is
     * returned, initialization will stop immediately after returning from this
     * function and the sensors will not be started.
     */
    virtual bool postSensorCreationHook() { return true; }

    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::LPS28DFW> lps28dfw;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    std::unique_ptr<Boardcore::MPXH6115A> staticPressure;
    std::unique_ptr<Boardcore::MPXH6115A> dynamicPressure;
    std::unique_ptr<Boardcore::RotatedIMU> rotatedImu;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    /**
     * Sensor creation and insertion need to happen separately to allow
     * integration with the HIL framework, which needs to intercept the sensors
     * after they are created but before they are inserted into the manager.
     */

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

    void staticPressureInit();
    void staticPressureCallback();

    void dynamicPressureInit();
    void dynamicPressureCallback();

    void rotatedImuInit();
    void rotatedImuCallback();

    bool sensorManagerInit();

    miosix::FastMutex baroCalibrationMutex;
    float baroCalibrationReference   = 0;
    bool useBaroCalibrationReference = false;

    miosix::FastMutex magCalibrationMutex;
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    size_t magCalibrationTaskId = 0;

    miosix::FastMutex gyroCalibrationMutex;
    Boardcore::BiasCorrector gyroCalibration;

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};

}  // namespace Payload
