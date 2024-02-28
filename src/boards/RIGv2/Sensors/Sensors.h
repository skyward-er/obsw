/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Sensors/SensorsData.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/MAX31856/MAX31856.h>
#include <sensors/SensorManager.h>

#include <atomic>
#include <functional>
#include <memory>
#include <vector>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIGv2
{

class Sensors : public Boardcore::Module
{
public:
    explicit Sensors(Boardcore::TaskScheduler &scheduler) : scheduler{scheduler}
    {
    }

    [[nodiscard]] bool start();

    bool isStarted();

    // Getters for raw data coming from sensors
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADC1LastSample();
    Boardcore::MAX31856Data getTc1LastSample();

    // Getters for processed data
    Boardcore::PressureData getVesselPress();
    Boardcore::PressureData getFillingPress();
    Boardcore::PressureData getTankTopPress();
    Boardcore::PressureData getTankBottomPress();
    Boardcore::LoadCellData getVesselWeight();
    Boardcore::LoadCellData getTankWeight();
    Boardcore::CurrentData getUmbilicalCurrent();
    Boardcore::CurrentData getServoCurrent();
    VoltageData getBatteryVoltage();

    void calibrate();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

private:
    void internalAdcInit(Boardcore::SensorManager::SensorMap_t &map);
    void internalAdcCallback();

    void adc1Init(Boardcore::SensorManager::SensorMap_t &map);
    void adc1Callback();

    void tc1Init(Boardcore::SensorManager::SensorMap_t &map);
    void tc1Callback();

    Boardcore::Logger &sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
    Boardcore::TaskScheduler &scheduler;

    std::atomic<float> vesselLcOffset{0.0f};
    std::atomic<float> tankLcOffset{0.0f};

    std::atomic<bool> started{false};
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::MAX31856> tc1;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv2