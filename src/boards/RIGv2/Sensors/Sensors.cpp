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

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    SensorManager::SensorMap_t map;
    internalAdcInit(map);
    adc1Init(map);
    tc1Init(map);
    vesselPressureInit(map);
    fillingPressureInit(map);
    topTankPressureInit(map);
    bottomTankPressureInit(map);
    vesselWeightInit(map);
    tankWeightInit(map);

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
    if (!internalAdc)
    {
        return {};
    }
    return internalAdc->getLastSample();
}

ADS131M08Data Sensors::getADC1LastSample()
{
    PauseKernelLock l;
    if (!adc1)
    {
        return {};
    }
    return adc1->getLastSample();
}

MAX31856Data Sensors::getTc1LastSample()
{
    PauseKernelLock l;
    if (!tc1)
    {
        return {};
    }

    return tc1->getLastSample();
}

PressureData Sensors::getVesselPress()
{
    PauseKernelLock l;
    if (!vesselPressure)
    {
        return {};
    }
    return vesselPressure->getLastSample();
}

PressureData Sensors::getFillingPress()
{
    PauseKernelLock l;
    if (!fillingPressure)
    {
        return {};
    }
    return fillingPressure->getLastSample();
}

PressureData Sensors::getTopTankPress()
{
    PauseKernelLock l;
    if (!topTankPressure)
    {
        return {};
    }
    return topTankPressure->getLastSample();
}

PressureData Sensors::getBottomTankPress()
{
    PauseKernelLock l;
    if (!bottomTankPressure)
    {
        return {};
    }
    return bottomTankPressure->getLastSample();
}

LoadCellData Sensors::getVesselWeight()
{
    PauseKernelLock l;
    if (!vesselWeight)
    {
        return {};
    }
    return vesselWeight->getLastSample();
}

LoadCellData Sensors::getTankWeight()
{
    PauseKernelLock l;
    if (!tankWeight)
    {
        return {};
    }
    return tankWeight->getLastSample();
}

CurrentData Sensors::getUmbilicalCurrent()
{
    return {TimestampTimer::getTimestamp(), 0.0};
}

CurrentData Sensors::getServoCurrent()
{
    auto sample = getADC1LastSample();

    float current =
        (sample.voltage[(int)Config::Sensors::ADC1_SERVO_CURRENT_CHANNEL] -
         Config::Sensors::SERVO_CURRENT_ZERO) *
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
            sample.voltage[(int)Config::Sensors::ADC1_VESSEL_LC_CHANNEL]);
        tankStats.add(
            sample.voltage[(int)Config::Sensors::ADC1_TANK_LC_CHANNEL]);

        Thread::sleep(Config::Sensors::LC_CALIBRATE_SAMPLE_PERIOD);
    }

    vesselLcOffset = vesselStats.getStats().mean;
    tankLcOffset   = tankStats.getStats().mean;
}

std::vector<SensorInfo> Sensors::getSensorInfos()
{
    return {
        manager->getSensorInfo(vesselWeight.get()),
        manager->getSensorInfo(tankWeight.get()),
        manager->getSensorInfo(vesselPressure.get()),
        manager->getSensorInfo(fillingPressure.get()),
        manager->getSensorInfo(topTankPressure.get()),
        manager->getSensorInfo(bottomTankPressure.get()),
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
    // Setup global configurations
    config.oversamplingRatio     = ADS131M08Defs::OversamplingRatio::OSR_8192;
    config.globalChopModeEnabled = true;

    // Disable all channels
    config.channelsConfig[0].enabled = false;
    config.channelsConfig[1].enabled = false;
    config.channelsConfig[2].enabled = false;
    config.channelsConfig[3].enabled = false;
    config.channelsConfig[4].enabled = false;
    config.channelsConfig[5].enabled = false;
    config.channelsConfig[6].enabled = false;
    config.channelsConfig[7].enabled = false;

    // Configure all required channels
    config.channelsConfig[(int)Config::Sensors::ADC1_VESSEL_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC1_FILLING_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC1_BOTTOM_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC1_TOP_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC1_VESSEL_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_32,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC1_TANK_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_32,
        .offset  = 0,
        .gain    = 1.0};

    adc1 = std::make_unique<ADS131M08>(modules.get<Buses>()->getADS131M08_1(),
                                       sensors::ADS131_1::cs::getPin(),
                                       spiConfig, config);

    SensorInfo info("ADS131M08_1", 500, [this]() { adc1Callback(); });
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

void Sensors::vesselPressureInit(Boardcore::SensorManager::SensorMap_t &map)
{
    vesselPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = adc1->getLastSample();
            return sample.getVoltage(Config::Sensors::ADC1_VESSEL_PT_CHANNEL);
        },
        Config::Sensors::ADC1_CH1_SHUNT_RESISTANCE,
        Config::Sensors::VESSEL_MAX_PRESSURE, Config::Sensors::PT_MIN_CURRENT,
        Config::Sensors::PT_MAX_CURRENT);

    SensorInfo info("VesselPressure", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { vesselPressureCallback(); });
    map.emplace(std::make_pair(vesselPressure.get(), info));
}

void Sensors::vesselPressureCallback()
{
    PressureData sample = vesselPressure->getLastSample();
    PTsData data{sample.pressureTimestamp, 1, sample.pressure};
    sdLogger.log(data);
}

void Sensors::fillingPressureInit(Boardcore::SensorManager::SensorMap_t &map)
{
    fillingPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = adc1->getLastSample();
            return sample.getVoltage(Config::Sensors::ADC1_FILLING_PT_CHANNEL);
        },
        Config::Sensors::ADC1_CH2_SHUNT_RESISTANCE,
        Config::Sensors::FILLING_MAX_PRESSURE, Config::Sensors::PT_MIN_CURRENT,
        Config::Sensors::PT_MAX_CURRENT);

    SensorInfo info("FillingPressure", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { fillingPressureCallback(); });
    map.emplace(std::make_pair(fillingPressure.get(), info));
}

void Sensors::fillingPressureCallback()
{
    PressureData sample = fillingPressure->getLastSample();
    PTsData data{sample.pressureTimestamp, 2, sample.pressure};
    sdLogger.log(data);
}

void Sensors::topTankPressureInit(Boardcore::SensorManager::SensorMap_t &map)
{
    topTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = adc1->getLastSample();
            return sample.getVoltage(Config::Sensors::ADC1_TOP_PT_CHANNEL);
        },
        Config::Sensors::ADC1_CH3_SHUNT_RESISTANCE,
        Config::Sensors::TANK_TOP_MAX_PRESSURE, Config::Sensors::PT_MIN_CURRENT,
        Config::Sensors::PT_MAX_CURRENT);

    SensorInfo info("TopTankPressure", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { topTankPressureCallback(); });
    map.emplace(std::make_pair(topTankPressure.get(), info));
}

void Sensors::topTankPressureCallback()
{
    PressureData sample = topTankPressure->getLastSample();
    PTsData data{sample.pressureTimestamp, 3, sample.pressure};
    sdLogger.log(data);
}

void Sensors::bottomTankPressureInit(Boardcore::SensorManager::SensorMap_t &map)
{
    bottomTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = adc1->getLastSample();
            return sample.getVoltage(Config::Sensors::ADC1_BOTTOM_PT_CHANNEL);
        },
        Config::Sensors::ADC1_CH4_SHUNT_RESISTANCE,
        Config::Sensors::TANK_BOTTOM_MAX_PRESSURE,
        Config::Sensors::PT_MIN_CURRENT, Config::Sensors::PT_MAX_CURRENT);

    SensorInfo info("BottomTankPressure", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { bottomTankPressureCallback(); });
    map.emplace(std::make_pair(bottomTankPressure.get(), info));
}

void Sensors::bottomTankPressureCallback()
{
    PressureData sample = bottomTankPressure->getLastSample();
    PTsData data{sample.pressureTimestamp, 4, sample.pressure};
    sdLogger.log(data);
}

void Sensors::vesselWeightInit(Boardcore::SensorManager::SensorMap_t &map)
{
    vesselWeight = std::make_unique<AnalogLoadCellSensor>(
        [this]()
        {
            auto sample = adc1->getLastSample();
            auto voltage =
                sample.getVoltage(Config::Sensors::ADC1_VESSEL_LC_CHANNEL);
            voltage.voltage -= vesselLcOffset;

            return voltage;
        },
        Config::Sensors::LC_VESSEL_SCALE);

    SensorInfo info("VesselWeight", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { vesselWeightCallback(); });
    map.emplace(std::make_pair(vesselWeight.get(), info));
}

void Sensors::vesselWeightCallback()
{
    LoadCellData sample = vesselWeight->getLastSample();
    LCsData data{sample.loadTimestamp, 1, sample.load};
    sdLogger.log(data);
}

void Sensors::tankWeightInit(Boardcore::SensorManager::SensorMap_t &map)
{
    tankWeight = std::make_unique<AnalogLoadCellSensor>(
        [this]()
        {
            auto sample = adc1->getLastSample();
            auto voltage =
                sample.getVoltage(Config::Sensors::ADC1_TANK_LC_CHANNEL);
            voltage.voltage -= tankLcOffset;

            return voltage;
        },
        Config::Sensors::LC_TANK_SCALE);

    SensorInfo info("TankWeight", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { tankWeightCallback(); });
    map.emplace(std::make_pair(tankWeight.get(), info));
}

void Sensors::tankWeightCallback()
{
    LoadCellData sample = tankWeight->getLastSample();
    LCsData data{sample.loadTimestamp, 2, sample.load};
    sdLogger.log(data);
}