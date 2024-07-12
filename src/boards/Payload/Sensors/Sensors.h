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
#include <Payload/Sensors/RotatedIMU/RotatedIMU.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/Pitot/Pitot.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN015PA.h>
#include <sensors/analog/pressure/honeywell/SSCMRNN030PA.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "SensorData.h"

namespace Payload
{
class Buses;
class BoardScheduler;

/**
 * @brief Manages all the sensors of the payload board.
 */
class Sensors : public Boardcore::InjectableWithDeps<BoardScheduler>
{
public:
    /**
     * @brief Constructs a new Sensors object. Requires a `Buses` object to be
     * passed to the constructor instead of using dependency injection because
     * the `Buses` module would not be injected yet at the time of construction.
     *
     * @param buses The `Buses` object to retrieve bus instances for the sensors
     */
    explicit Sensors(Buses& buses);

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
     * @brief Takes the result of the live magnetometer calibration and applies
     * it to the current calibration + writes it in the csv file
     *
     * @return true if the write was successful
     */
    bool writeMagCalibration();

    // Hardware Sensors
    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::LPS28DFWData getLPS28DFWLastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::UBXGPSData getUBXGPSLastSample();
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample();
    Boardcore::ADS131M08Data getADS131M08LastSample();
    Boardcore::InternalADCData getInternalADCLastSample();

    // Software Sensors
    Boardcore::HSCMRNN015PAData getStaticPressureLastSample();
    Boardcore::SSCMRNN030PAData getDynamicPressureLastSample();
    Boardcore::PitotData getPitotLastSample();
    RotatedIMUData getIMULastSample();

    // Processed values
    Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();
    Boardcore::BatteryVoltageSensorData getCamBatteryVoltageLastSample();
    Boardcore::MagnetometerData getCalibratedMagnetometerLastSample();

    void pitotSetReferenceAltitude(float altitude);
    void pitotSetReferenceTemperature(float temperature);

    /**
     * @brief Returns information about all sensors managed by this class
     */
    std::vector<Boardcore::SensorInfo> getSensorInfo();

private:
    /**
     * Sensor creation and insertion need to happen separately to allow
     * integration with the HIL framework, which needs to intercept the sensors
     * after they are created but before they are inserted into the manager.
     */

    void lps22dfCreate(Buses& buses);
    void lps22dfInsert(Boardcore::SensorManager::SensorMap_t& map);

    void lps28dfwCreate(Buses& buses);
    void lps28dfwInsert(Boardcore::SensorManager::SensorMap_t& map);

    void h3lis331dlCreate(Buses& buses);
    void h3lis331dlInsert(Boardcore::SensorManager::SensorMap_t& map);

    void lis2mdlCreate(Buses& buses);
    void lis2mdlInsert(Boardcore::SensorManager::SensorMap_t& map);

    void ubxgpsCreate(Buses& buses);
    void ubxgpsInsert(Boardcore::SensorManager::SensorMap_t& map);

    void lsm6dsrxCreate(Buses& buses);
    void lsm6dsrxInsert(Boardcore::SensorManager::SensorMap_t& map);

    void ads131m08Create(Buses& buses);
    void ads131m08Insert(Boardcore::SensorManager::SensorMap_t& map);

    void internalAdcCreate(Buses& buses);
    void internalAdcInsert(Boardcore::SensorManager::SensorMap_t& map);

    void staticPressureCreate(Buses& buses);
    void staticPressureInsert(Boardcore::SensorManager::SensorMap_t& map);

    void dynamicPressureCreate(Buses& buses);
    void dynamicPressureInsert(Boardcore::SensorManager::SensorMap_t& map);

    void pitotCreate(Buses& buses);
    void pitotInsert(Boardcore::SensorManager::SensorMap_t& map);

    void imuCreate(Buses& buses);
    void imuInsert(Boardcore::SensorManager::SensorMap_t& map);

    bool magCalibrationInit(Boardcore::TaskScheduler& scheduler);

protected:
    // Hardware sensor instances
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::LPS28DFW> lps28dfw;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    // Software sensor instances
    std::unique_ptr<Boardcore::HSCMRNN015PA> staticPressure;
    std::unique_ptr<Boardcore::SSCMRNN030PA> dynamicPressure;
    std::unique_ptr<Boardcore::Pitot> pitot;
    std::unique_ptr<Payload::RotatedIMU> imu;

private:
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;

    miosix::FastMutex calibrationMutex;

    std::unique_ptr<Boardcore::SensorManager> manager;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};

}  // namespace Payload
