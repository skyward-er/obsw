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
#include <RIGv2/Sensors/SensorsData.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/MAX31856/MAX31856.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/TrafagPressureSensor.h>
#include <sensors/analog/TwoPointAnalogLoadCell.h>

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

    void calibrate();

    bool isStarted();

    // Getters for raw data coming from sensors
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADC1LastSample();
    Boardcore::MAX31856Data getTc1LastSample();

    // Getters for processed data
    Boardcore::PressureData getVesselPressLastSample();
    Boardcore::PressureData getFillingPressLastSample();
    Boardcore::PressureData getTopTankPressLastSample();
    Boardcore::PressureData getBottomTankPressLastSample();
    Boardcore::PressureData getCCPressLastSample();
    Boardcore::TemperatureData getTankTempLastSample();
    Boardcore::LoadCellData getVesselWeightLastSample();
    Boardcore::LoadCellData getTankWeightLastSample();
    Boardcore::CurrentData getUmbilicalCurrentLastSample();
    Boardcore::CurrentData getServoCurrentLastSample();
    Boardcore::VoltageData getBatteryVoltageLastSample();
    Boardcore::VoltageData getMotorBatteryVoltageLastSample();

    Boardcore::PressureData getCanTopTankPressLastSample();
    Boardcore::PressureData getCanBottomTankPressLastSample();
    Boardcore::PressureData getCanCCPressLastSample();
    Boardcore::TemperatureData getCanTankTempLastSample();
    Boardcore::VoltageData getCanMotorBatteryVoltageLastSample();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

    void setCanTopTankPress(Boardcore::PressureData data);
    void setCanBottomTankPress(Boardcore::PressureData data);
    void setCanCCPress(Boardcore::PressureData data);
    void setCanTankTemp(Boardcore::TemperatureData data);
    void setCanMotorBatteryVoltage(Boardcore::VoltageData data);
    void switchToCanSensors();

private:
    void vesselPressureInit();
    void vesselPressureCallback();

    void fillingPressureInit();
    void fillingPressureCallback();

    void topTankPressureInit();
    void topTankPressureCallback();

    void bottomTankPressureInit();
    void bottomTankPressureCallback();

    void vesselWeightInit();
    void vesselWeightCallback();

    void tankWeightInit();
    void tankWeightCallback();

    void internalAdcInit();
    void internalAdcCallback();

    void adc1Init();
    void adc1Callback();

    void tc1Init();
    void tc1Callback();

    bool sensorManagerInit();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};

    std::atomic<bool> useCanData{false};
    miosix::FastMutex canMutex;
    Boardcore::PressureData canCCPressure;
    Boardcore::PressureData canBottomTankPressure;
    Boardcore::PressureData canTopTankPressure;
    Boardcore::TemperatureData canTankTemperature;
    Boardcore::VoltageData canMotorBatteryVoltage;

    // Analog sensors
    std::unique_ptr<Boardcore::TrafagPressureSensor> vesselPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> fillingPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> topTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> bottomTankPressure;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> vesselWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> tankWeight;

    // Digital sensors
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::MAX31856> tc1;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv2
