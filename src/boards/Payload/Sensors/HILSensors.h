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
#include <HIL_sensors/HILSensor.h>
#include <HIL_sensors/HILTemperature.h>
#include <HIL_sensors/HILTimestampManagement.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/Pitot/Pitot.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "Sensors.h"

namespace Payload
{

class HILSensors : public Sensors
{
public:
    explicit HILSensors(Boardcore::TaskScheduler* sched);

    [[nodiscard]] bool start() override;

    // Sensor getters
    Boardcore::LPS22DFData getLPS22DFLastSample() override;
    Boardcore::LPS28DFWData getLPS28DFW_1LastSample() override;
    Boardcore::LPS28DFWData getLPS28DFW_2LastSample() override;
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample() override;
    Boardcore::LIS2MDLData getLIS2MDLLastSample() override;
    Boardcore::UBXGPSData getGPSLastSample() override;
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample() override;
    Boardcore::ADS131M08Data getADS131M08LastSample() override;

    Boardcore::HSCMRNN015PAData getStaticPressureLastSample() override;
    Boardcore::SSCMRNN030PAData getDynamicPressureLastSample() override;
    Boardcore::PitotData getPitotLastSample() override;

private:
    // Init and callbacks methods
    void temperatureInit();

    void lps22dfInit() override;
    void lps22dfCallback() override;

    void lps28dfw_1Init();
    void lps28dfw_1Callback();

    void lps28dfw_2Init();
    void lps28dfw_2Callback();

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

    void staticPressureInit() override;
    void staticPressureCallback() override;

    void dynamicPressureInit() override;
    void dynamicPressureCallback() override;

    void pitotInit() override;
    void pitotCallback() override;

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
    Boardcore::Pitot* pitot          = nullptr;
    HILBarometer* staticPressure     = nullptr;
    HILBarometer* dynamicPressure    = nullptr;
    // Boardcore::ADS131M08* ads131m08  = nullptr;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("HILSensors");
};
}  // namespace Payload