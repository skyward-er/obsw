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

#include "Sensors.h"

#include <RIGv2/Buses.h>
#include <RIGv2/Configs/SensorsConfig.h>
#include <interfaces-impl/hwmapping.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace miosix;
using namespace RIGv2;

float pressureFromVoltage(float voltage, float shuntResistance,
                          float minCurrent, float maxCurrent, float maxPressure)
{
    // First convert voltage to current [mA]
    float current = (voltage / shuntResistance) * 1000.0f;

    // Convert to a value between [0, 1] based on the min and max current
    float value = (current - minCurrent) / (maxCurrent - minCurrent);

    // Finally remap to the range [0, maxPressure]
    return value * maxPressure;
}

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    SensorManager::SensorMap_t map;
    internalAdcInit(map);
    adc1Init(map);
    tc1Init(map);

    manager = std::make_unique<SensorManager>(map, &scheduler);
    if (!manager->start())
    {
        LOG_ERR(logger, "Failed to start SensorManager");
        return false;
    }

    started = true;
    return true;
}

InternalADCData Sensors::getInternalADCLastSample()
{
    PauseKernelLock l;
    return internalAdc->getLastSample();
}

ADS131M08Data Sensors::getADC1LastSample()
{
    PauseKernelLock l;
    return adc1->getLastSample();
}

MAX31856Data Sensors::getTc1LastSample()
{
    PauseKernelLock l;
    return tc1->getLastSample();
}

PressureData Sensors::getVesselPress()
{
    auto sample = getADC1LastSample();

    float pressure = pressureFromVoltage(
        sample.voltage[Config::Sensors::ADC1_VESSEL_PT_CHANNEL],
        Config::Sensors::ADC1_CH1_SHUNT_RESISTANCE,
        Config::Sensors::PT_MIN_CURRENT, Config::Sensors::PT_MAX_CURRENT,
        Config::Sensors::VESSEL_MAX_PRESSURE);
    return {sample.timestamp, pressure};
}

PressureData Sensors::getFillingPress()
{
    auto sample = getADC1LastSample();

    float pressure = pressureFromVoltage(
        sample.voltage[Config::Sensors::ADC1_FILLING_PT_CHANNEL],
        Config::Sensors::ADC1_CH2_SHUNT_RESISTANCE,
        Config::Sensors::PT_MIN_CURRENT, Config::Sensors::PT_MAX_CURRENT,
        Config::Sensors::FILLING_MAX_PRESSURE);
    return {sample.timestamp, pressure};
}

PressureData Sensors::getTankTopPress()
{
    auto sample = getADC1LastSample();

    float pressure = pressureFromVoltage(
        sample.voltage[Config::Sensors::ADC1_TOP_PT_CHANNEL],
        Config::Sensors::ADC1_CH3_SHUNT_RESISTANCE,
        Config::Sensors::PT_MIN_CURRENT, Config::Sensors::PT_MAX_CURRENT,
        Config::Sensors::FILLING_MAX_PRESSURE);
    return {sample.timestamp, pressure};
}

PressureData Sensors::getTankBottomPress()
{
    auto sample = getADC1LastSample();

    float pressure = pressureFromVoltage(
        sample.voltage[Config::Sensors::ADC1_BOTTOM_PT_CHANNEL],
        Config::Sensors::ADC1_CH4_SHUNT_RESISTANCE,
        Config::Sensors::PT_MIN_CURRENT, Config::Sensors::PT_MAX_CURRENT,
        Config::Sensors::FILLING_MAX_PRESSURE);
    return {sample.timestamp, pressure};
}

LoadCellData Sensors::getVesselWeight()
{
    auto sample = getADC1LastSample();
    float calibratedVoltage =
        sample.voltage[Config::Sensors::ADC1_VESSEL_LC_CHANNEL] -
        vesselLcOffset;

    return {sample.timestamp, calibratedVoltage};
}

LoadCellData Sensors::getTankWeight()
{
    auto sample = getADC1LastSample();
    float calibratedVoltage =
        sample.voltage[Config::Sensors::ADC1_TANK_LC_CHANNEL] - tankLcOffset;

    return {sample.timestamp, calibratedVoltage};
}

CurrentData Sensors::getUmbilicalCurrent()
{
    return {TimestampTimer::getTimestamp(), 0.0};
}

CurrentData Sensors::getServoCurrent()
{
    auto sample = getADC1LastSample();

    float current = (sample.voltage[Config::Sensors::ADC1_SERVO_CURRENT_CHANNEL] - Config::Sensors::SERVO_CURRENT_ZERO) *
                    Config::Sensors::SERVO_CURRENT_SCALE;
    // Current reading are flipped
    return {sample.timestamp, -current / 5.0f * 50.0f};
}

VoltageData Sensors::getBatteryVoltage()
{
    auto sample = getInternalADCLastSample();

    float voltage = sample.voltage[14] * Config::Sensors::BATTERY_VOLTAGE_SCALE;
    return {sample.timestamp, voltage};
}

void Sensors::calibrate()
{
    Stats vesselStats, tankStats;

    for (unsigned int i = 0; i < Config::Sensors::LC_CALIBRATE_SAMPLE_COUNT;
         i++)
    {
        auto sample = getADC1LastSample();

        vesselStats.add(
            sample.voltage[Config::Sensors::ADC1_VESSEL_LC_CHANNEL]);
        tankStats.add(sample.voltage[Config::Sensors::ADC1_TANK_LC_CHANNEL]);

        Thread::sleep(Config::Sensors::LC_CALIBRATE_SAMPLE_PERIOD);
    }

    vesselLcOffset = vesselStats.getStats().mean;
    tankLcOffset   = tankStats.getStats().mean;
}

std::vector<SensorInfo> Sensors::getSensorInfos()
{
    return {
        manager->getSensorInfo(internalAdc.get()),
        manager->getSensorInfo(adc1.get()),
        manager->getSensorInfo(tc1.get()),
    };
}

void Sensors::internalAdcInit(Boardcore::SensorManager::SensorMap_t &map)
{
    internalAdc = std::make_unique<InternalADC>(ADC1);

    internalAdc->enableChannel(InternalADC::CH9);
    internalAdc->enableChannel(InternalADC::CH11);
    internalAdc->enableChannel(InternalADC::CH14);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();

    SensorInfo info("InternalAdc", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { internalAdcCallback(); });
    map.emplace(internalAdc.get(), info);
}

void Sensors::internalAdcCallback()
{
    InternalADCData sample = internalAdc->getLastSample();
    sdLogger.log(sample);
}

void Sensors::adc1Init(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config = {};
    config.channelsConfig[Config::Sensors::ADC1_VESSEL_LC_CHANNEL].pga =
        ADS131M08Defs::PGA::PGA_32;
    config.channelsConfig[Config::Sensors::ADC1_TANK_LC_CHANNEL].pga =
        ADS131M08Defs::PGA::PGA_32;
    config.oversamplingRatio     = ADS131M08Defs::OversamplingRatio::OSR_8192;
    config.globalChopModeEnabled = true;

    adc1 = std::make_unique<ADS131M08>(modules.get<Buses>()->getADS131M08_1(),
                                       sensors::ADS131_1::cs::getPin(),
                                       spiConfig, config);

    SensorInfo info("ADS131M08_1", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { adc1Callback(); });
    map.emplace(std::make_pair(adc1.get(), info));
}

void Sensors::adc1Callback()
{
    ADS131M08Data sample = adc1->getLastSample();

    ADCsData data{sample.timestamp,  1,
                  sample.voltage[0], sample.voltage[1],
                  sample.voltage[2], sample.voltage[3],
                  sample.voltage[4], sample.voltage[5],
                  sample.voltage[6], sample.voltage[7]};

    sdLogger.log(data);
}

void Sensors::tc1Init(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = MAX31856::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    tc1 =
        std::make_unique<MAX31856>(modules.get<Buses>()->getMAX31856_1(),
                                   sensors::MAX31856_1::cs::getPin(), spiConfig,
                                   MAX31856::ThermocoupleType::K_TYPE);

    SensorInfo info("MAX31856_1", Config::Sensors::TC_SAMPLE_PERIOD,
                    [this]() { tc1Callback(); });
    map.emplace(std::make_pair(tc1.get(), info));
}

void Sensors::tc1Callback()
{
    MAX31856Data sample = tc1->getLastSample();

    TCsData data{sample.temperatureTimestamp, 1, sample.temperature,
                 sample.coldJunctionTemperature};

    sdLogger.log(data);
}