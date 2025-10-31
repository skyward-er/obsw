/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Pietro Bortolus
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
#include <common/canbus/MotorStatus.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/MAX31856/MAX31856.h>
#include <sensors/SensorManager.h>
#include <sensors/analog/AnalogEncoder.h>
#include <sensors/analog/TrafagPressureSensor.h>
#include <sensors/analog/TwoPointAnalogLoadCell.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>
#include <functional>
#include <memory>
#include <vector>

namespace RIGv2
{

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler,
                                                     Common::MotorStatus>
{
public:
    Sensors() {}

    [[nodiscard]] bool start();

    void calibrate();

    void calibrateLoadcells();

    void calibrateEncoders();

    bool isStarted();

    // Getters for raw data coming from sensors
    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADC1LastSample();
    Boardcore::ADS131M08Data getADC2LastSample();

    // Getters for processed data
    Boardcore::PressureData getOxVesselPressure();
    Boardcore::PressureData getPrzVessel1Pressure();
    Boardcore::PressureData getPrzVessel2Pressure();
    Boardcore::PressureData getPrzTankPressure();
    Boardcore::PressureData getRegulatorPressure();
    Boardcore::PressureData getOxTankPressure();
    Boardcore::PressureData getFuelTankPressure();

    Boardcore::LoadCellData getOxVesselWeight();
    Boardcore::LoadCellData getRocketWeight();
    Boardcore::LoadCellData getOxTankWeight();

    Boardcore::ServoPositionData getFuelValvePosition();
    Boardcore::ServoPositionData getOxValvePosition();

    Boardcore::CurrentData getUmbilicalCurrent();
    Boardcore::CurrentData getServoCurrent();
    Boardcore::VoltageData getBatteryVoltage();

    /**
     * @brief Returns the Ox tank bottom pressure, either sampled directly by
     * the RIG or received by the Motor board over CAN bus.
     */
    Boardcore::PressureData getOxTankBottomPressureDirectOrCan();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

private:
    void oxVesselPressureInit();
    void oxVesselPressureCallback();

    void przVessel1PressureInit();
    void przVessel1PressureCallback();

    void przVessel2PressureInit();
    void przVessel2PressureCallback();

    void regulatorPressureInit();
    void regulatorPressureCallback();

    void przTankPressureInit();
    void przTankPressureCallback();

    void oxTankPressureInit();
    void oxTankPressureCallback();

    void fuelTankPressureInit();
    void fuelTankPressureCallback();

    void oxVesselWeightInit();
    void oxVesselWeightCallback();

    void rocketWeightInit();
    void rocketWeightCallback();

    void oxTankWeightInit();
    void oxTankWeightCallback();

    void oxValvePositionInit();
    void oxValvePositionCallback();

    void fuelValvePositionInit();
    void fuelValvePositionCallback();

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

    // Analog sensors
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxVesselPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przVessel1Pressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przVessel2Pressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> regulatorPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> przTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> oxTankPressure;
    std::unique_ptr<Boardcore::TrafagPressureSensor> fuelTankPressure;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> oxVesselWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> rocketWeight;
    std::unique_ptr<Boardcore::TwoPointAnalogLoadCell> oxTankWeight;
    std::unique_ptr<Boardcore::AnalogEncoder> oxValvePosition;
    std::unique_ptr<Boardcore::AnalogEncoder> fuelValvePosition;

    // Digital sensors
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::ADS131M08> adc2;
    // std::unique_ptr<Boardcore::MAX31856> tc1;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv2
