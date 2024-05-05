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

#include <Payload/Configs/HILSimulationConfig.h>
#include <sensors/HILSensors/IncludeHILSensors.h>
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

    // Sensor getters
    Boardcore::LPS22DFData getLPS22DFLastSample() override;
    Boardcore::LPS28DFWData getLPS28DFW_1LastSample() override;
    Boardcore::LPS28DFWData getLPS28DFW_2LastSample() override;
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample() override;
    Boardcore::LIS2MDLData getLIS2MDLLastSample() override;
    Boardcore::UBXGPSData getGPSLastSample() override;
    Boardcore::LSM6DSRXData getLSM6DSRXLastSample() override;

    Boardcore::HSCMRNN015PAData getStaticPressureLastSample() override;
    Boardcore::SSCMRNN030PAData getDynamicPressureLastSample() override;

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

    void staticPressureInit() override;
    void staticPressureCallback() override;

    void dynamicPressureInit() override;
    void dynamicPressureCallback() override;

    void imuInit() override;
    void imuCallback() override;

    // HILSensors instances
    HILConfig::PayloadHILTemperature* temperature      = nullptr;
    HILConfig::PayloadHILBarometer* lps22df            = nullptr;
    HILConfig::PayloadHILBarometer* lps28dfw_1         = nullptr;
    HILConfig::PayloadHILBarometer* lps28dfw_2         = nullptr;
    HILConfig::PayloadHILAccelerometer* h3lis331dl     = nullptr;
    HILConfig::PayloadHILMagnetometer* lis2mdl         = nullptr;
    HILConfig::PayloadHILGps* ubxgps                   = nullptr;
    HILConfig::PayloadHILAccelerometer* lsm6dsrx_accel = nullptr;
    HILConfig::PayloadHILGyroscope* lsm6dsrx_gyro      = nullptr;
    HILConfig::PayloadHILBarometer* staticPressure     = nullptr;
    HILConfig::PayloadHILBarometer* dynamicPressure    = nullptr;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("HILSensors");
};
}  // namespace Payload