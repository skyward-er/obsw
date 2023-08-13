/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Emilio Corigliano
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

#include <HIL_sensors/HILAccelerometer.h>
#include <HIL_sensors/HILBarometer.h>
#include <HIL_sensors/HILGps.h>
#include <HIL_sensors/HILGyroscope.h>
#include <HIL_sensors/HILMagnetometer.h>
#include <HIL_sensors/HILPitot.h>
#include <HIL_sensors/HILSensor.h>
#include <HIL_sensors/HILTemperature.h>
#include <HIL_sensors/HILTimestampManagement.h>
#include <sensors/SensorManager.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "Sensors.h"

namespace Main
{

class HILSensors : public Sensors
{
public:
    explicit HILSensors(Boardcore::TaskScheduler* sched);

    [[nodiscard]] bool start() override;

    /**
     * @brief Stops the sensor manager
     * @warning Stops the passed scheduler
     */
    void stop() override;

    /**
     * @brief Returns if all the sensors are started successfully
     */
    bool isStarted() override;

    /**
     * @brief Calibrates the sensors with an offset
     */
    void calibrate() override;

    // Sensor getters
    Boardcore::LPS22DFData getLPS22DFLastSample() override;
    Boardcore::LPS28DFWData getLPS28DFW_1LastSample() override;
    Boardcore::LPS28DFWData getLPS28DFW_2LastSample() override;
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample() override;
    Boardcore::LIS2MDLData getLIS2MDLLastSample() override;
    Boardcore::UBXGPSData getGPSLastSample() override;
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample() override;
    Boardcore::ADS131M08Data getADS131M08LastSample() override;
    Boardcore::MPXH6400AData getDeploymentPressureLastSample() override;
    Boardcore::HSCMRNN015PAData getStaticPressure1LastSample() override;
    Boardcore::HSCMRNN015PAData getStaticPressure2LastSample() override;
    RotatedIMUData getIMULastSample() override;

private:
    // Init and callbacks methods
    void temperatureInit();

    void lps22dfInit() override;
    void lps22dfCallback() override;

    void lps28dfw_1Init() override;
    void lps28dfw_1Callback() override;

    void lps28dfw_2Init() override;
    void lps28dfw_2Callback() override;

    void h3lis331dlInit() override;
    void h3lis331dlCallback() override;

    void lis2mdlInit() override;
    void lis2mdlCallback() override;

    void ubxgpsInit() override;
    void ubxgpsCallback() override;

    void lsm6dsrxInit() override;
    void lsm6dsrxCallback() override;

    void ads131m08Init() override;
    void ads131m08Callback() override;

    void deploymentPressureInit() override;
    void deploymentPressureCallback() override;

    void staticPressure1Init() override;
    void staticPressure1Callback() override;

    void staticPressure2Init() override;
    void staticPressure2Callback() override;

    void imuInit() override;
    void imuCallback() override;

    // HILSensors instances
    HILTemperature* temperature      = nullptr;
    HILBarometer* lps22df            = nullptr;
    HILBarometer* lps28dfw_1         = nullptr;
    HILBarometer* lps28dfw_2         = nullptr;
    HILAccelerometer* h3lis331dl     = nullptr;
    HILMagnetometer* lis2mdl         = nullptr;
    HILGps* ubxgps                   = nullptr;
    HILAccelerometer* lsm6dsrx_accel = nullptr;
    HILGyroscope* lsm6dsrx_gyro      = nullptr;
    // Boardcore::ADS131M08* ads131m08  = nullptr;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("HILSensors");
};
}  // namespace Main