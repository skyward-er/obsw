/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <RIG/Buses.h>
#include <RIG/Configs/SensorsConfig.h>
#include <RIG/Sensors/ADCsData.h>
#include <RIG/Sensors/LoadCellsData.h>
#include <RIG/Sensors/Sensors.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace miosix;
using namespace std;

namespace RIG
{
ADS131M04Data Sensors::getADC1LastSample()
{
    PauseKernelLock l;
    return adc1->getLastSample();
}

ADS131M04Data Sensors::getADC2LastSample()
{
    PauseKernelLock l;
    return adc2->getLastSample();
}

HX711Data Sensors::getTankWeight()
{
    PauseKernelLock l;
    HX711Data sample = loadCell2->getLastSample();
    sample.load -= offsetLoadCell2;
    sample.load = -sample.load;
    return sample;
}

HX711Data Sensors::getVesselWeight()
{
    PauseKernelLock l;
    HX711Data sample = loadCell1->getLastSample();
    sample.load -= offsetLoadCell1;
    return sample;
}

TemperatureData Sensors::getThermocoupleLastSample()
{
    PauseKernelLock l;
    return ((!canSensorFlag) ? thermocouple->getLastSample()
                             : canTankTemperature);
}

PressureData Sensors::getFillingPress()
{
    // It is important to call the getter method to separate the critical
    // section from the calculation ones
    ADS131M04Data lastSample = getADC1LastSample();

    // Calculate the current and then based on linear relation calculate the
    // pressure
    float current =
        lastSample.voltage[1] / Config::Sensors::ADC1_CH1_SHUNT_RESISTANCE;

    // Convert the current in mA
    current = current * 1000;

    // Calculate the pressure given the linear formula 4..20mA for
    // 0..max_pressure

    // shift the current to let the line pass through 0 point
    current = current - Config::Sensors::SENSOR_MIN_CURRENT;

    float pressure = (current / (Config::Sensors::SENSOR_MAX_CURRENT -
                                 Config::Sensors::SENSOR_MIN_CURRENT)) *
                     Config::Sensors::FILLING_MAX_PRESSURE;

    return PressureData{lastSample.timestamp, pressure};
}

PressureData Sensors::getTankTopPress()
{
    // It is important to call the getter method to separate the critical
    // section from the calculation ones
    ADS131M04Data lastSample = getADC1LastSample();

    // Calculate the current and then based on linear relation calculate the
    // pressure
    float current =
        lastSample.voltage[3] / Config::Sensors::ADC1_CH2_SHUNT_RESISTANCE;

    // Convert the current in mA
    current = current * 1000;

    // Calculate the pressure given the linear formula 4..20mA for
    // 0..max_pressure

    // shift the current to let the line pass through 0 point
    current = current - Config::Sensors::SENSOR_MIN_CURRENT;

    float pressure = (current / (Config::Sensors::SENSOR_MAX_CURRENT -
                                 Config::Sensors::SENSOR_MIN_CURRENT)) *
                     Config::Sensors::TANK_TOP_MAX_PRESSURE;

    return ((!canSensorFlag) ? PressureData{lastSample.timestamp, pressure}
                             : canTopTankPressure);
}

PressureData Sensors::getTankBottomPress()
{
    // It is important to call the getter method to separate the critical
    // section from the calculation ones
    ADS131M04Data lastSample = getADC1LastSample();

    // Calculate the current and then based on linear relation calculate the
    // pressure
    float current =
        lastSample.voltage[2] / Config::Sensors::ADC1_CH3_SHUNT_RESISTANCE;

    // Convert the current in mA
    current = current * 1000;

    // Calculate the pressure given the linear formula 4..20mA for
    // 0..max_pressure

    // shift the current to let the line pass through 0 point
    current = current - Config::Sensors::SENSOR_MIN_CURRENT;

    float pressure = (current / (Config::Sensors::SENSOR_MAX_CURRENT -
                                 Config::Sensors::SENSOR_MIN_CURRENT)) *
                     Config::Sensors::TANK_BOTTOM_MAX_PRESSURE;

    return ((!canSensorFlag) ? PressureData{lastSample.timestamp, pressure}
                             : canBottomTankPressure);
}

PressureData Sensors::getVesselPress()
{
    // It is important to call the getter method to separate the critical
    // section from the calculation ones
    ADS131M04Data lastSample = getADC1LastSample();

    // Calculate the current and then based on linear relation calculate the
    // pressure
    float current =
        lastSample.voltage[0] / Config::Sensors::ADC1_CH4_SHUNT_RESISTANCE;

    // Convert the current in mA
    current = current * 1000;

    // Calculate the pressure given the linear formula 4..20mA for
    // 0..max_pressure

    // shift the current to let the line pass through 0 point
    current = current - Config::Sensors::SENSOR_MIN_CURRENT;

    float pressure = (current / (Config::Sensors::SENSOR_MAX_CURRENT -
                                 Config::Sensors::SENSOR_MIN_CURRENT)) *
                     Config::Sensors::VESSEL_MAX_PRESSURE;

    return PressureData{lastSample.timestamp, pressure};
}

BatteryVoltageSensorData Sensors::getBatteryVoltage()
{
    // Get the analog ADC2 reading
    ADS131M04Data lastSample = getADC2LastSample();

    // Raw reading
    float voltage =
        Config::Sensors::VOLTAGE_CONVERSION_FACTOR * lastSample.voltage[2];

    BatteryVoltageSensorData result{};

    result.voltage          = voltage;
    result.voltageTimestamp = lastSample.timestamp;
    return result;
}

CurrentData Sensors::getCurrent()
{
    // Get the analog ADC2 reading
    ADS131M04Data lastSample = getADC2LastSample();

    // Raw voltage reading
    float voltage =
        Config::Sensors::CURRENT_CONVERSION_FACTOR * lastSample.voltage[3];

    // 2.5V is half the dynamic
    float current = -((voltage - 2.52) /
                      Config::Sensors::VOLTAGE_CURRENT_CONVERSION_FACTOR);

    CurrentData result{};

    result.current          = current;
    result.currentTimestamp = lastSample.timestamp;
    return result;
}

PressureData Sensors::getCCPress()
{
    PauseKernelLock lock;
    return canCCPressure;
}

CurrentData Sensors::getMainCurrent()
{
    PauseKernelLock lock;
    return (!canSensorFlag) ? CurrentData{} : canMainCurrent;
}

CurrentData Sensors::getPayloadCurrent()
{
    PauseKernelLock lock;
    return (!canSensorFlag) ? CurrentData{} : canPayloadCurrent;
}

CurrentData Sensors::getMotorCurrent()
{
    PauseKernelLock lock;
    return (!canSensorFlag) ? CurrentData{} : canMotorCurrent;
}

BatteryVoltageSensorData Sensors::getMotorBatteryVoltage()
{
    PauseKernelLock lock;
    return (!canSensorFlag) ? BatteryVoltageSensorData{} : canMotorVoltage;
}

void Sensors::setCCPressure(PressureData data)
{
    PauseKernelLock lock;
    canSensorFlag                   = true;
    canCCPressure                   = data;
    canCCPressure.pressureTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setBottomTankPressure(PressureData data)
{
    PauseKernelLock lock;
    canSensorFlag                           = true;
    canBottomTankPressure                   = data;
    canBottomTankPressure.pressureTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setTopTankPressure(PressureData data)
{
    PauseKernelLock lock;
    canSensorFlag                        = true;
    canTopTankPressure                   = data;
    canTopTankPressure.pressureTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setTankTemperature(TemperatureData data)
{
    PauseKernelLock lock;
    canSensorFlag                           = true;
    canTankTemperature                      = data;
    canTankTemperature.temperatureTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setMotorCurrent(CurrentData data)
{
    PauseKernelLock lock;
    canSensorFlag                    = true;
    canMotorCurrent                  = data;
    canMotorCurrent.currentTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setMainCurrent(CurrentData data)
{
    PauseKernelLock lock;
    canSensorFlag                   = true;
    canMainCurrent                  = data;
    canMainCurrent.currentTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setPayloadCurrent(CurrentData data)
{
    PauseKernelLock lock;
    canSensorFlag                      = true;
    canPayloadCurrent                  = data;
    canPayloadCurrent.currentTimestamp = TimestampTimer::getTimestamp();
}

void Sensors::setMotorVoltage(BatteryVoltageSensorData data)
{
    PauseKernelLock lock;
    canSensorFlag                    = true;
    canMotorVoltage                  = data;
    canMotorVoltage.voltageTimestamp = TimestampTimer::getTimestamp();
}

Sensors::Sensors(TaskScheduler* sched) : scheduler{sched} {}

bool Sensors::start()
{
    // Init all the sensors
    adc1Init();
    adc2Init();
    loadCell1Init();
    loadCell2Init();
    thermocoupleInit();

    // Create the sensors manager with the desired scheduler and the sensors map
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start();
}

void Sensors::stop() { manager->stop(); }

bool Sensors::isStarted() { return manager->areAllSensorsInitialized(); }

void Sensors::calibrate()
{
    Stats sample1, sample2;

    // Take some samples of the 2 load cells
    for (int i = 0; i < 10; i++)
    {
        // DO NOT USE THE GETTERS BECAUSE THE OFFSET IS ALREADY APPLIED
        {
            PauseKernelLock lock;
            sample1.add(loadCell1->getLastSample().load);
            sample2.add(loadCell2->getLastSample().load);
        }

        // Wait for some reasonable time
        Thread::sleep(Config::Sensors::LOAD_CELL_SAMPLE_PERIOD * 3);
    }

    offsetLoadCell1 = sample1.getStats().mean;
    offsetLoadCell2 = sample2.getStats().mean;
}

void Sensors::adc1Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig config;
    config.mode         = SPI::Mode::MODE_0;
    config.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M04::Config sensorConfig;
    sensorConfig.oversamplingRatio = ADS131M04Defs::OversamplingRatio::OSR_8192;
    sensorConfig.globalChopModeEnabled = true;

    adc1 = new ADS131M04(modules.get<Buses>()->spi1,
                         sensors::ADS131_1::cs::getPin(), config, sensorConfig);

    // Config and enter the sensors info to feed to the sensors manager
    SensorInfo info("ADS131_1", Config::Sensors::ADC_SAMPLE_PERIOD,
                    bind(&Sensors::adc1Callback, this));
    sensorMap.emplace(make_pair(adc1, info));
    LOG_INFO(logger, "Initialized ADC1");
}
void Sensors::adc1Callback()
{
    ADS131M04Data sample = adc1->getLastSample();

    ADCsData data{sample.timestamp,  1,
                  sample.voltage[0], sample.voltage[1],
                  sample.voltage[2], sample.voltage[3]};

    SDlogger.log(data);
}

void Sensors::adc2Init()
{
    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig config;
    config.mode         = SPI::Mode::MODE_0;
    config.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M04::Config sensorConfig;
    sensorConfig.oversamplingRatio = ADS131M04Defs::OversamplingRatio::OSR_8192;
    sensorConfig.globalChopModeEnabled = true;

    adc2 = new ADS131M04(modules.get<Buses>()->spi1,
                         sensors::ADS131_2::cs::getPin(), config, sensorConfig);

    // Config and enter the sensors info to feed to the sensors manager
    SensorInfo info("ADS131_2", Config::Sensors::ADC_SAMPLE_PERIOD,
                    bind(&Sensors::adc2Callback, this));
    sensorMap.emplace(make_pair(adc2, info));
    LOG_INFO(logger, "Initialized ADC2");
}
void Sensors::adc2Callback()
{
    ADS131M04Data sample = adc2->getLastSample();

    ADCsData data{sample.timestamp,  2,
                  sample.voltage[0], sample.voltage[1],
                  sample.voltage[2], sample.voltage[3]};

    SDlogger.log(data);
}

void Sensors::loadCell1Init()
{
    ModuleManager& modules = ModuleManager::getInstance();
    loadCell1 =
        new HX711(modules.get<Buses>()->spi2, sensors::HX711_1::sck::getPin());

    loadCell1->setOffset(Config::Sensors::LOAD_CELL1_OFFSET);
    loadCell1->setScale(Config::Sensors::LOAD_CELL1_SCALE);

    // Config and enter the sensors info to feed to the sensors manager
    SensorInfo info("HX711_1", Config::Sensors::LOAD_CELL_SAMPLE_PERIOD,
                    bind(&Sensors::loadCell1Callback, this));
    sensorMap.emplace(make_pair(loadCell1, info));
    LOG_INFO(logger, "Initialized loadCell1");
}
void Sensors::loadCell1Callback()
{
    HX711Data sample = loadCell1->getLastSample();

    LoadCellsData data;

    data.cellNumber    = 1;
    data.load          = sample.load;
    data.loadTimestamp = sample.loadTimestamp;

    SDlogger.log(data);
}

void Sensors::loadCell2Init()
{
    ModuleManager& modules = ModuleManager::getInstance();
    loadCell2 =
        new HX711(modules.get<Buses>()->spi6, sensors::HX711_2::sck::getPin());

    loadCell2->setOffset(Config::Sensors::LOAD_CELL2_OFFSET);
    loadCell2->setScale(Config::Sensors::LOAD_CELL2_SCALE);

    // Config and enter the sensors info to feed to the sensors manager
    SensorInfo info("HX711_2", Config::Sensors::LOAD_CELL_SAMPLE_PERIOD,
                    bind(&Sensors::loadCell2Callback, this));
    sensorMap.emplace(make_pair(loadCell2, info));
    LOG_INFO(logger, "Initialized loadCell2");
}
void Sensors::loadCell2Callback()
{
    HX711Data sample = loadCell2->getLastSample();

    LoadCellsData data;

    data.cellNumber    = 2;
    data.load          = sample.load;
    data.loadTimestamp = sample.loadTimestamp;

    SDlogger.log(data);
}

void Sensors::thermocoupleInit()
{
    ModuleManager& modules = ModuleManager::getInstance();
    thermocouple           = new MAX31855(modules.get<Buses>()->spi1,
                                          sensors::MAX31855::cs::getPin());

    // Config and enter the sensors info to feed to the sensors manager
    SensorInfo info("MAX31855", Config::Sensors::THERMOCOUPLE_SAMPLE_PERIOD,
                    bind(&Sensors::thermocoupleCallback, this));
    sensorMap.emplace(make_pair(thermocouple, info));
    LOG_INFO(logger, "Initialized thermocouple");
}
void Sensors::thermocoupleCallback()
{
    SDlogger.log(thermocouple->getLastSample());
}

}  // namespace RIG