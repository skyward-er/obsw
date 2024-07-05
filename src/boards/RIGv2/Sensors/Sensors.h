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

#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Buses.h>
#include <RIGv2/Sensors/AnalogLoadCellSensor.h>
#include <RIGv2/Sensors/SensorsData.h>
#include <RIGv2/Sensors/TrafagPressureSensor.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/MAX31856/MAX31856.h>
#include <sensors/SensorManager.h>

#include <atomic>
#include <functional>
#include <memory>
#include <utils/ModuleManager/ModuleManager.hpp>
#include <vector>

namespace RIGv2
{

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler>
{
public:
    Sensors() {}

    [[nodiscard]] bool start();

    bool isStarted();

    // Getters for raw data coming from sensors
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADC1LastSample();
    Boardcore::MAX31856Data getTc1LastSample();

    // Getters for processed data
    Boardcore::PressureData getVesselPress();
    Boardcore::PressureData getFillingPress();
    Boardcore::PressureData getTopTankPress();
    Boardcore::PressureData getBottomTankPress();
    Boardcore::PressureData getCCPress();
    Boardcore::TemperatureData getTankTemp();
    Boardcore::LoadCellData getVesselWeight();
    Boardcore::LoadCellData getTankWeight();
    Boardcore::CurrentData getUmbilicalCurrent();
    Boardcore::CurrentData getServoCurrent();
    Boardcore::VoltageData getBatteryVoltage();

    void setCanTopTankPress(Boardcore::PressureData data);
    void setCanBottomTankPress(Boardcore::PressureData data);
    void setCanCCPress(Boardcore::PressureData data);
    void setCanTankTemp(Boardcore::TemperatureData data);

    void calibrate();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

private:
    void vesselPressureInit(Boardcore::SensorManager::SensorMap_t &map);
    void vesselPressureCallback();

    void fillingPressureInit(Boardcore::SensorManager::SensorMap_t &map);
    void fillingPressureCallback();

    void topTankPressureInit(Boardcore::SensorManager::SensorMap_t &map);
    void topTankPressureCallback();

    void bottomTankPressureInit(Boardcore::SensorManager::SensorMap_t &map);
    void bottomTankPressureCallback();

    void vesselWeightInit(Boardcore::SensorManager::SensorMap_t &map);
    void vesselWeightCallback();

    void tankWeightInit(Boardcore::SensorManager::SensorMap_t &map);
    void tankWeightCallback();

    void internalAdcInit(Boardcore::SensorManager::SensorMap_t &map);
    void internalAdcCallback();

    void adc1Init(Boardcore::SensorManager::SensorMap_t &map);
    void adc1Callback();

    void tc1Init(Boardcore::SensorManager::SensorMap_t &map);
    void tc1Callback();

    Boardcore::Logger &sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<float> vesselLcOffset{0.0f};
    std::atomic<float> tankLcOffset{0.0f};

    std::atomic<bool> started{false};

    std::atomic<bool> useCanData{false};
    miosix::FastMutex canMutex;
    Boardcore::PressureData canCCPressure;
    Boardcore::PressureData canBottomTankPressure;
    Boardcore::PressureData canTopTankPressure;
    Boardcore::TemperatureData canTankTemperature;

    // Analog sensors
    std::unique_ptr<TrafagPressureSensor> vesselPressure;
    std::unique_ptr<TrafagPressureSensor> fillingPressure;
    std::unique_ptr<TrafagPressureSensor> topTankPressure;
    std::unique_ptr<TrafagPressureSensor> bottomTankPressure;
    std::unique_ptr<AnalogLoadCellSensor> vesselWeight;
    std::unique_ptr<AnalogLoadCellSensor> tankWeight;

    // Digital sensors
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::MAX31856> tc1;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv2