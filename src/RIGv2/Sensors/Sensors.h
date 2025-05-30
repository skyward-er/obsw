/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>
#include <functional>
#include <memory>
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
    Boardcore::ADS131M08Data getADC2LastSample();
    Boardcore::MAX31856Data getTc1LastSample();

    // Getters for processed data
    Boardcore::PressureData getOxVesselPressure();
    Boardcore::PressureData getOxFillingPressure();
    Boardcore::PressureData getN2Vessel1Pressure();
    Boardcore::PressureData getN2Vessel2Pressure();
    Boardcore::PressureData getN2FillingPressure();
    Boardcore::PressureData getOxTankBottomPressure();
    Boardcore::PressureData getN2TankPressure();
    Boardcore::PressureData getCombustionChamberPressure();

    Boardcore::TemperatureData getOxTankTemperature();
    Boardcore::LoadCellData getOxVesselWeight();
    Boardcore::LoadCellData getRocketWeight();
    Boardcore::LoadCellData getOxTankWeight();

    Boardcore::CurrentData getUmbilicalCurrent();
    Boardcore::CurrentData getServoCurrent();
    Boardcore::VoltageData getBatteryVoltage();
    Boardcore::VoltageData getMotorBatteryVoltage();

    Boardcore::PressureData getCanOxTankBottomPressure();
    Boardcore::PressureData getCanOxTankTopPressure();
    Boardcore::PressureData getCanN2TankPressure();
    Boardcore::PressureData getCanCombustionChamberPressure();
    Boardcore::TemperatureData getCanTankTemperature();
    Boardcore::VoltageData getCanMotorBatteryVoltage();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

    void setCanOxTankBottomPressure(Boardcore::PressureData data);
    void setCanOxTankTopPressure(Boardcore::PressureData data);
    void setCanN2TankPressure(Boardcore::PressureData data);
    void setCanCombustionChamberPressure(Boardcore::PressureData data);
    void setCanOxTankTemperature(Boardcore::TemperatureData data);
    void setCanMotorBatteryVoltage(Boardcore::VoltageData data);
    void switchToCanSensors();

private:
    void oxVesselPressureInit();
    void oxVesselPressureCallback();

    void oxFillingPressureInit();
    void oxFillingPressureCallback();

    void n2Vessel1PressureInit();
    void n2Vessel1PressureCallback();

    void n2Vessel2PressureInit();
    void n2Vessel2PressureCallback();

    void n2FillingPressureInit();
    void n2FillingPressureCallback();

    void oxTankBottomPressureInit();
    void oxTankBottomPressureCallback();

    void n2TankPressureInit();
    void n2TankPressureCallback();

    void oxVesselWeightInit();
    void oxVesselWeightCallback();

    void rocketWeightInit();
    void rocketWeightCallback();

    void oxTankWeightInit();
    void oxTankWeightCallback();

    void internalAdcInit();
    void internalAdcCallback();

    void adc1Init();
    void adc1Callback();

    void adc2Init();
    void adc2Callback();

    void tc1Init();
    void tc1Callback();

    bool sensorManagerInit();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};

    std::atomic<bool> useCanData{false};
    miosix::FastMutex canMutex;

    Boardcore::PressureData canOxTankBottomPressure;
    Boardcore::PressureData canOxTankTopPressure;
    Boardcore::PressureData canN2TankPressure;
    Boardcore::PressureData canCombustionChamberPressure;
    // TODO: N2 tank pressure from CAN
    Boardcore::TemperatureData canOxTankTemperature;
    Boardcore::VoltageData canMotorBatteryVoltage;

    // Analog sensors
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxVesselPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxFillingPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> n2Vessel1Pressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> n2Vessel2Pressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> n2FillingPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxTankBottomPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> n2TankPressure;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> oxVesselWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> rocketWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> oxTankWeight;

    // Digital sensors
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::ADS131M08> adc2;
    std::unique_ptr<Boardcore::MAX31856> tc1;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv2
