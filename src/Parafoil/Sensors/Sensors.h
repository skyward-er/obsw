/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <Parafoil/Configs/SensorsConfig.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/BatteryVoltageSensor.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "SensorData.h"

namespace Parafoil
{
class BoardScheduler;
class Buses;
class FlightStatsRecorder;

/**
 * @brief Manages all the sensors of the parafoil board.
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

    Boardcore::BMX160Data getBMX160LastSample();
    Boardcore::BMX160WithCorrectionData getBMX160WithCorrectionLastSample();
    Boardcore::H3LIS331DLData getH3LISLastSample();
    Boardcore::LIS3MDLData getLIS3MDLLastSample();
    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::ADS131M08Data getADS131LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();

    Boardcore::BatteryVoltageSensorData getBatteryVoltage();

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

    std::unique_ptr<Boardcore::BMX160> bmx160;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS3MDL> lis3mdl;
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    std::unique_ptr<Boardcore::BMX160WithCorrection> bmx160WithCorrection;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    /**
     * Sensors initialization and callback functions.
     */

    void bmx160Init();
    void bmx160Callback();

    void bmx160WithCorrectionInit();
    void bmx160WithCorrectionCallback();

    void h3lisInit();
    void h3lisCallback();

    void lis3mdlInit();
    void lis3mdlCallback();

    void lps22Init();
    void lps22Callback();

    void ubxGpsInit();
    void ubxGpsCallback();

    void ads131Init();
    void ads131Callback();

    void internalADCInit();
    void internalADCCallback();

    bool sensorManagerInit();

    // Live calibration of the magnetomer
    miosix::FastMutex magCalibrationMutex;
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    size_t magCalibrationTaskId = 0;

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};

}  // namespace Parafoil
