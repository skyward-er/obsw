/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Pietro Bortolus
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

#include <RIGv3/BoardScheduler.h>
#include <RIGv3/Buses.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/AnalogEncoder.h>
#include <sensors/analog/TrafagPressureSensor.h>
#include <sensors/analog/TwoPointAnalogLoadCell.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>
#include <functional>
#include <memory>

namespace RIGv3
{

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler>
{
public:
    [[nodiscard]] bool start();

    void calibrate();

    void calibrateLoadcells();

    void calibrateEncoders();

    bool isStarted();

    // Getters for raw data coming from sensors
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADC0LastSample();
    Boardcore::ADS131M08Data getADC1LastSample();
    Boardcore::ADS131M08Data getADC2LastSample();
    Boardcore::ADS131M08Data getADC3LastSample();

    // Getters for processed data
    Boardcore::PressureData getPrzVessel1Pressure();
    Boardcore::PressureData getPrzVessel2Pressure();
    Boardcore::PressureData getPrzFillingPressure();
    Boardcore::PressureData getOxVesselPressure();
    Boardcore::PressureData getOxFillingPressure();

    Boardcore::PressureData getPrzTankPressure();
    Boardcore::PressureData getOxRegOutPressure();
    Boardcore::PressureData getFuelRegOutPressure();
    Boardcore::PressureData getOxTankPressure();
    Boardcore::PressureData getFuelTankPressure();
    Boardcore::PressureData getIgniterChamberPressure();
    Boardcore::PressureData getMainChamberPressure();

    Boardcore::LoadCellData getOxVesselWeight();
    Boardcore::LoadCellData getRocketWeight();
    Boardcore::LoadCellData getOxTankWeight();

    Boardcore::ServoPositionData getMainOxPosition();
    Boardcore::ServoPositionData getMainFuelPosition();
    Boardcore::ServoPositionData getOxRegPosition();
    Boardcore::ServoPositionData getFuelRegPosition();

    Boardcore::CurrentData getUmbilicalCurrent();
    Boardcore::CurrentData getServoCurrent();
    Boardcore::VoltageData getBatteryVoltage();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

private:
    void przVessel1PressureInit();
    void przVessel1PressureCallback();

    void przVessel2PressureInit();
    void przVessel2PressureCallback();

    void przFillingPressureInit();
    void przFillingPressureCallback();

    void oxVesselPressureInit();
    void oxVesselPressureCallback();

    void oxFillingPressureInit();
    void oxFillingPressureCallback();

    void oxRegOutPressureInit();
    void oxRegOutPressureCallback();

    void fuelRegOutPressureInit();
    void fuelRegOutPressureCallback();

    void przTankPressureInit();
    void przTankPressureCallback();

    void oxTankPressureInit();
    void oxTankPressureCallback();

    void fuelTankPressureInit();
    void fuelTankPressureCallback();

    void igniterChamberPressureInit();
    void igniterChamberPressureCallback();

    void mainChamberPressureInit();
    void mainChamberPressureCallback();

    void oxVesselWeightInit();
    void oxVesselWeightCallback();

    void rocketWeightInit();
    void rocketWeightCallback();

    void oxTankWeightInit();
    void oxTankWeightCallback();

    void mainOxPositionInit();
    void mainOxPositionCallback();

    void mainFuelPositionInit();
    void mainFuelPositionCallback();

    void oxRegPositionInit();
    void oxRegPositionCallback();

    void fuelRegPositionInit();
    void fuelRegPositionCallback();

    void internalAdcInit();
    void internalAdcCallback();

    void adc0Init();
    void adc0Callback();

    void adc1Init();
    void adc1Callback();

    void adc2Init();
    void adc2Callback();

    void adc3Init();
    void adc3Callback();

    bool sensorManagerInit();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};

    // Digital sensors
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::ADS131M08> adc0;  // Adc for the cart
    std::unique_ptr<Boardcore::ADS131M08> adc1;  // Adc for the cart
    std::unique_ptr<Boardcore::ADS131M08> adc2;  // Adc for the test stand
    std::unique_ptr<Boardcore::ADS131M08> adc3;  // Adc for the test stand

    // Analog sensors
    std::unique_ptr<Boardcore::TrafagPressureSensor> przVessel1Pressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przVessel2Pressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przFillingPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxVesselPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxFillingPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxRegOutPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> fuelRegOutPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> fuelTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> igniterChamberPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> mainChamberPressure;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> oxVesselWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> rocketWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> oxTankWeight;
    std::unique_ptr<Boardcore::AnalogEncoder> mainOxPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> mainFuelPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> oxRegPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> fuelRegPosition;

    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv3
