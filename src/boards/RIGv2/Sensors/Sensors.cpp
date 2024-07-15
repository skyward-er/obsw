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
    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getSensorsScheduler();

    SensorManager::SensorMap_t map;
    if (Config::Sensors::InternalADC::ENABLED)
    {
        internalAdcInit(map);
    }
    if (Config::Sensors::ADS131M08::ENABLED)
    {
        adc1Init(map);
    }
    if (Config::Sensors::MAX31856::ENABLED)
    {
        tc1Init(map);
    }
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
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}

ADS131M08Data Sensors::getADC1LastSample()
{
    return adc1 ? adc1->getLastSample() : ADS131M08Data{};
}

MAX31856Data Sensors::getTc1LastSample()
{
    return tc1 ? tc1->getLastSample() : MAX31856Data{};
}

PressureData Sensors::getVesselPress()
{
    return vesselPressure ? vesselPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getFillingPress()
{
    return fillingPressure ? fillingPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getTopTankPress()
{
    if (useCanData)
    {
        Lock<FastMutex> lock{canMutex};
        return canTopTankPressure;
    }
    else
    {
        return topTankPressure ? topTankPressure->getLastSample()
                               : PressureData{};
    }
}

PressureData Sensors::getBottomTankPress()
{
    if (useCanData)
    {
        Lock<FastMutex> lock{canMutex};
        return canBottomTankPressure;
    }
    else
    {
        return bottomTankPressure ? bottomTankPressure->getLastSample()
                                  : PressureData{};
    }
}

PressureData Sensors::getCCPress()
{
    if (useCanData)
    {
        Lock<FastMutex> lock{canMutex};
        return canCCPressure;
    }
    else
    {
        return PressureData{};
    }
}

TemperatureData Sensors::getTankTemp()
{
    if (useCanData)
    {
        Lock<FastMutex> lock{canMutex};
        return canTankTemperature;
    }
    else
    {
        return getTc1LastSample();
    }
}

LoadCellData Sensors::getVesselWeight()
{
    if (vesselWeight)
    {
        return vesselWeight->getLastSample();
    }
    else
    {
        return {};
    }
}

LoadCellData Sensors::getTankWeight()
{
    if (tankWeight)
    {
        return tankWeight->getLastSample();
    }
    else
    {
        return {};
    }
}

CurrentData Sensors::getUmbilicalCurrent()
{
    return {TimestampTimer::getTimestamp(), -1.0};
}

CurrentData Sensors::getServoCurrent()
{
    auto sample = getADC1LastSample();

    float current =
        (sample
             .voltage[(int)Config::Sensors::ADS131M08::SERVO_CURRENT_CHANNEL] -
         Config::Sensors::ADS131M08::SERVO_CURRENT_ZERO) *
        Config::Sensors::ADS131M08::SERVO_CURRENT_SCALE;
    // Current reading are flipped
    return {sample.timestamp, -current / 5.0f * 50.0f};
}

VoltageData Sensors::getBatteryVoltage()
{
    auto sample = getInternalADCLastSample();

    float voltage =
        sample.voltage[(
            int)Config::Sensors::InternalADC::BATTERY_VOLTAGE_CHANNEL] *
        Config::Sensors::InternalADC::BATTERY_VOLTAGE_SCALE;
    return {sample.timestamp, voltage};
}

VoltageData Sensors::getMotorBatteryVoltage()
{
    if (useCanData)
    {
        Lock<FastMutex> lock{canMutex};
        return canMotorBatteryVoltage;
    }
    else
    {
        return VoltageData{};
    }
}

void Sensors::setCanTopTankPress(Boardcore::PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTopTankPressure = data;
    useCanData         = true;
}

void Sensors::setCanBottomTankPress(Boardcore::PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canBottomTankPressure = data;
    useCanData            = true;
}

void Sensors::setCanCCPress(Boardcore::PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canCCPressure = data;
    useCanData    = true;
}

void Sensors::setCanTankTemp(Boardcore::TemperatureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTankTemperature = data;
    useCanData         = true;
}

void Sensors::setCanMotorBatteryVoltage(Boardcore::VoltageData data)
{
    Lock<FastMutex> lock{canMutex};
    canMotorBatteryVoltage = data;
    useCanData             = true;
}

void Sensors::calibrate()
{
    Stats vesselStats, tankStats;

    for (unsigned int i = 0;
         i < Config::Sensors::LoadCell::CALIBRATE_SAMPLE_COUNT; i++)
    {
        // Tank readings WITHOUT offsets
        vesselStats.add(vesselWeight->getLastSample().load);
        tankStats.add(tankWeight->getLastSample().load);

        Thread::sleep(Config::Sensors::LoadCell::CALIBRATE_SAMPLE_PERIOD);
    }

    vesselWeight->updateOffset(vesselStats.getStats().mean);
    tankWeight->updateOffset(tankStats.getStats().mean);
}

std::vector<SensorInfo> Sensors::getSensorInfos()
{
    if (manager)
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
    else
    {
        return {};
    }
}

void Sensors::internalAdcInit(Boardcore::SensorManager::SensorMap_t &map)
{
    internalAdc = std::make_unique<InternalADC>(ADC1);

    internalAdc->enableChannel(InternalADC::CH9);
    internalAdc->enableChannel(InternalADC::CH11);
    internalAdc->enableChannel(
        Config::Sensors::InternalADC::BATTERY_VOLTAGE_CHANNEL);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();

    SensorInfo info("InternalAdc", Config::Sensors::InternalADC::SAMPLE_PERIOD,
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
    config.channelsConfig[(int)Config::Sensors::ADS131M08::VESSEL_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::FILLING_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::BOTTOM_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::TOP_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::SERVO_CURRENT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::VESSEL_LC_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_32,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::TANK_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_32,
        .offset  = 0,
        .gain    = 1.0};

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08_1(),
                                       sensors::ADS131_1::cs::getPin(),
                                       spiConfig, config);

    SensorInfo info("ADS131M08_1", Config::Sensors::ADS131M08::SAMPLE_PERIOD,
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

    // LOG_INFO(logger, "{:.4}\t{:.4}\t{:.4}\t{:.4}",
    //          (sample.voltage[0] / Config::Sensors::ADC1_CH1_SHUNT_RESISTANCE)
    //          * 1000.0f, (sample.voltage[1] /
    //          Config::Sensors::ADC1_CH2_SHUNT_RESISTANCE) * 1000.0f,
    //          (sample.voltage[2] / Config::Sensors::ADC1_CH3_SHUNT_RESISTANCE)
    //          * 1000.0f, (sample.voltage[3] /
    //          Config::Sensors::ADC1_CH4_SHUNT_RESISTANCE) * 1000.0f);

    sdLogger.log(data);
}

void Sensors::tc1Init(SensorManager::SensorMap_t &map)
{
    SPIBusConfig spiConfig = MAX31856::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    tc1 = std::make_unique<MAX31856>(
        getModule<Buses>()->getMAX31856_1(), sensors::MAX31856_1::cs::getPin(),
        spiConfig, MAX31856::ThermocoupleType::K_TYPE);

    SensorInfo info("MAX31856_1", Config::Sensors::MAX31856::SAMPLE_PERIOD,
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
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::VESSEL_PT_CHANNEL);
        },
        Config::Sensors::Trafag::VESSEL_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::VESSEL_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);

    SensorInfo info("VesselPressure", Config::Sensors::ADS131M08::SAMPLE_PERIOD,
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
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::FILLING_PT_CHANNEL);
        },
        Config::Sensors::Trafag::FILLING_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::FILLING_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);

    SensorInfo info("FillingPressure",
                    Config::Sensors::ADS131M08::SAMPLE_PERIOD,
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
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::TOP_PT_CHANNEL);
        },
        Config::Sensors::Trafag::TANK_TOP_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::TANK_TOP_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);

    SensorInfo info("TopTankPressure",
                    Config::Sensors::ADS131M08::SAMPLE_PERIOD,
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
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::BOTTOM_PT_CHANNEL);
        },
        Config::Sensors::Trafag::TANK_BOTTOM_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::TANK_BOTTOM_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);

    SensorInfo info("BottomTankPressure",
                    Config::Sensors::ADS131M08::SAMPLE_PERIOD,
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
    vesselWeight = std::make_unique<TwoPointAnalogLoadCell>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::VESSEL_LC_CHANNEL);
        },
        Config::Sensors::LoadCell::VESSEL_P0_VOLTAGE,
        Config::Sensors::LoadCell::VESSEL_P0_MASS,
        Config::Sensors::LoadCell::VESSEL_P1_VOLTAGE,
        Config::Sensors::LoadCell::VESSEL_P1_MASS);

    SensorInfo info("VesselWeight", Config::Sensors::ADS131M08::SAMPLE_PERIOD,
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
    tankWeight = std::make_unique<TwoPointAnalogLoadCell>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::TANK_LC_CHANNEL);
        },
        Config::Sensors::LoadCell::TANK_P0_VOLTAGE,
        Config::Sensors::LoadCell::TANK_P0_MASS,
        Config::Sensors::LoadCell::TANK_P1_VOLTAGE,
        Config::Sensors::LoadCell::TANK_P1_MASS);

    SensorInfo info("TankWeight", Config::Sensors::ADS131M08::SAMPLE_PERIOD,
                    [this]() { tankWeightCallback(); });
    map.emplace(std::make_pair(tankWeight.get(), info));
}

void Sensors::tankWeightCallback()
{
    LoadCellData sample = tankWeight->getLastSample();
    LCsData data{sample.loadTimestamp, 2, sample.load};
    sdLogger.log(data);
}