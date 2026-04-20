/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Fabrizio Monti, Niccolò Betto, Riccardo Sironi
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

#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <drivers/adc/InternalADC.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/AnalogEncoder.h>
#include <sensors/analog/TrafagPressureSensor.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>
#include <vector>

namespace Motor
{

class Actuators;

class Sensors
    : public Boardcore::InjectableWithDeps<Buses, BoardScheduler, Actuators>
{
public:
    Sensors() {}

    [[nodiscard]] bool start();

    void calibrate();

    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data Sensors::getADC1LastSample();
    Boardcore::ADS131M08Data Sensors::getADC2LastSample();
    Boardcore::PressureData getCCPressure();
    Boardcore::PressureData getOxTankPressure();
    Boardcore::PressureData getFuelTankPressure();
    Boardcore::PressureData getPrzTankPressure();
    Boardcore::PressureData getRegulatorOutOxPressure();
    Boardcore::PressureData getRegulatorOutFuelPressure();
    Boardcore::PressureData getIgniterPressure();

    Boardcore::VoltageData getBatteryVoltage();
    Boardcore::CurrentData getCurrentConsumption();

    Boardcore::ServoPositionData getMainOxPosition();
    Boardcore::ServoPositionData getMainFuelPosition();
    Boardcore::ServoPositionData getPrzOxPosition();
    Boardcore::ServoPositionData getPrzFuelPosition();
    Boardcore::ServoPositionData getVentingOxPosition();
    Boardcore::ServoPositionData getVentingFuelPosition();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

protected:
    virtual bool postSensorCreationHook() { return true; }

    Boardcore::TaskScheduler& getSensorsScheduler();

    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::ADS131M08> adc2;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    // Analog sensors
    std::unique_ptr<Boardcore::TrafagPressureSensor> mainCCPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> fuelTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> regOutOxPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> regOutFuelPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> ignCCPressure;

    std::unique_ptr<Boardcore::AnalogEncoder> mainOxPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> mainFuelPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> przOxPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> przFuelPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> ventingOxPosition;
    std::unique_ptr<Boardcore::AnalogEncoder> ventingFuelPosition;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:
    void internalAdcInit();
    void internalAdcCallback();

    void adc1Init();
    void adc1Callback();

    void adc2Init();
    void adc2Callback();

    void regulatorOutOxPressureInit();
    void regulatorOutOxPressureCallback();

    void regulatorOutFuelPressureInit();
    void regulatorOutFuelPressureCallback();

    void oxTankPressureInit();
    void oxTankPressureCallback();

    void fuelTankPressureInit();
    void fuelTankPressureCallback();

    void przTankPressureInit();
    void przTankPressureCallback();

    void ignCCPressureInit();
    void ignCCPressureCallback();

    void mainCCPressureInit();
    void mainCCPressureCallback();

    void mainOxPositionInit();
    void mainOxPositionCallback();

    void mainFuelPositionInit();
    void mainFuelPositionCallback();

    void przOxPositionInit();
    void przOxPositionCallback();

    void przFuelPositionInit();
    void przFuelPositionCallback();

    void ventingOxPositionInit();
    void ventingOxPositionCallback();

    void ventingFuelPositionInit();
    void ventingFuelPositionCallback();

    bool sensorManagerInit();

    // TODO Da implementare. Capire bene cosa serve per il safety venting
    void checkOxTankOverpressure();
    // The last time point when the OX tank was ok (below threshold)
    std::chrono::steady_clock::time_point oxTankPressureOkTime = {};

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
};

}  // namespace Motor
