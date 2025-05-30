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

#include "Sensors.h"

#include <RIGv2/Configs/SensorsConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <interfaces-impl/hwmapping.h>

#include <chrono>

using namespace std::chrono;
using namespace Boardcore;
using namespace miosix;
using namespace RIGv2;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

    if (Config::Sensors::ADC_1::ENABLED)
    {
        adc1Init();
        oxVesselPressureInit();
        oxFillingPressureInit();
        n2Vessel1PressureInit();
        n2Vessel2PressureInit();
        n2FillingPressureInit();
        oxVesselWeightInit();
        rocketWeightInit();
        oxTankWeightInit();
    }

    if (Config::Sensors::ADC_2::ENABLED)
    {
        adc2Init();
        oxTankBottomPressureInit();
        n2TankPressureInit();
    }

    if (Config::Sensors::MAX31856::ENABLED)
        tc1Init();

    if (!sensorManagerInit())
    {
        LOG_ERR(logger, "Failed to init SensorManager");
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

ADS131M08Data Sensors::getADC2LastSample()
{
    return adc2 ? adc2->getLastSample() : ADS131M08Data{};
}

MAX31856Data Sensors::getTc1LastSample()
{
    return tc1 ? tc1->getLastSample() : MAX31856Data{};
}

PressureData Sensors::getOxVesselPressure()
{
    return oxVesselPressure ? oxVesselPressure->getLastSample()
                            : PressureData{};
}

PressureData Sensors::getOxFillingPressure()
{
    return oxFillingPressure ? oxFillingPressure->getLastSample()
                             : PressureData{};
}

PressureData Sensors::getN2Vessel1Pressure()
{
    return n2Vessel1Pressure ? n2Vessel1Pressure->getLastSample()
                             : PressureData{};
}

PressureData Sensors::getN2Vessel2Pressure()
{
    return n2Vessel2Pressure ? n2Vessel2Pressure->getLastSample()
                             : PressureData{};
}

PressureData Sensors::getN2FillingPressure()
{
    return n2FillingPressure ? n2FillingPressure->getLastSample()
                             : PressureData{};
}

PressureData Sensors::getOxTankBottomPressure()
{
    if (useCanData)
    {
        return getCanOxTankBottomPressure();
    }
    else
    {
        return oxTankBottomPressure ? oxTankBottomPressure->getLastSample()
                                    : PressureData{};
    }
}

PressureData Sensors::getN2TankPressure()
{
    if (useCanData)
    {
        return getCanN2TankPressure();
    }
    else
    {
        return n2TankPressure ? n2TankPressure->getLastSample()
                              : PressureData{};
    }
}

PressureData Sensors::getCombustionChamberPressure()
{
    if (useCanData)
        return getCanCombustionChamberPressure();
    else
        return PressureData{};
}

TemperatureData Sensors::getOxTankTemperature()
{
    if (useCanData)
        return getCanTankTemperature();
    else
        return getTc1LastSample();
}

LoadCellData Sensors::getOxVesselWeight()
{
    return oxVesselWeight ? oxVesselWeight->getLastSample() : LoadCellData{};
}

LoadCellData Sensors::getRocketWeight()
{
    return rocketWeight ? rocketWeight->getLastSample() : LoadCellData{};
}

LoadCellData Sensors::getOxTankWeight()
{
    return oxTankWeight ? oxTankWeight->getLastSample() : LoadCellData{};
}

CurrentData Sensors::getUmbilicalCurrent()
{
    // TODO: Implement umbilical current
    return {};
}

CurrentData Sensors::getServoCurrent()
{
    auto sample = getADC1LastSample();

    float current =
        (sample.voltage[(int)Config::Sensors::ADC_1::SERVO_CURRENT_CHANNEL] -
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
        return getCanMotorBatteryVoltage();
    else
        return VoltageData{};
}

PressureData Sensors::getCanN2TankPressure()
{
    Lock<FastMutex> lock{canMutex};
    return canN2TankPressure;
}

PressureData Sensors::getCanOxTankBottomPressure()
{
    Lock<FastMutex> lock{canMutex};
    return canOxTankBottomPressure;
}

PressureData Sensors::getCanOxTankTopPressure()
{
    Lock<FastMutex> lock{canMutex};
    return canOxTankTopPressure;
}

PressureData Sensors::getCanCombustionChamberPressure()
{
    Lock<FastMutex> lock{canMutex};
    return canCombustionChamberPressure;
}

TemperatureData Sensors::getCanTankTemperature()
{
    Lock<FastMutex> lock{canMutex};
    return canOxTankTemperature;
}

VoltageData Sensors::getCanMotorBatteryVoltage()
{
    Lock<FastMutex> lock{canMutex};
    return canMotorBatteryVoltage;
}

void Sensors::setCanOxTankBottomPressure(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canOxTankBottomPressure = data;
}

void Sensors::setCanOxTankTopPressure(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canOxTankTopPressure = data;
}

void Sensors::setCanN2TankPressure(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canN2TankPressure = data;
}

void Sensors::setCanCombustionChamberPressure(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canCombustionChamberPressure = data;
}

void Sensors::setCanOxTankTemperature(TemperatureData data)
{
    Lock<FastMutex> lock{canMutex};
    canOxTankTemperature = data;
}

void Sensors::setCanMotorBatteryVoltage(VoltageData data)
{
    Lock<FastMutex> lock{canMutex};
    canMotorBatteryVoltage = data;
}

void Sensors::switchToCanSensors() { useCanData = true; }

void Sensors::calibrate()
{
    Stats oxVesselStats, oxTankStats;

    for (unsigned int i = 0;
         i < Config::Sensors::LoadCell::CALIBRATE_SAMPLE_COUNT; i++)
    {
        // Tank readings WITHOUT offsets
        oxVesselStats.add(oxVesselWeight->getLastSample().load);
        oxTankStats.add(oxTankWeight->getLastSample().load);

        Thread::sleep(
            milliseconds{Config::Sensors::LoadCell::CALIBRATE_SAMPLE_PERIOD}
                .count());
    }

    oxVesselWeight->updateOffset(oxVesselStats.getStats().mean);
    oxTankWeight->updateOffset(oxTankStats.getStats().mean);
}

std::vector<SensorInfo> Sensors::getSensorInfos()
{
    if (manager)
    {
        std::vector<SensorInfo> infos{};

#define PUSH_SENSOR_INFO(instance, name)                         \
    if (instance)                                                \
        infos.push_back(manager->getSensorInfo(instance.get())); \
    else                                                         \
        infos.push_back(SensorInfo{name, 0ns, nullptr, false})

        PUSH_SENSOR_INFO(adc1, "ADS131M08_1");
        PUSH_SENSOR_INFO(adc2, "ADS131M08_2");
        PUSH_SENSOR_INFO(tc1, "MAX31856_1");
        PUSH_SENSOR_INFO(internalAdc, "InternalADC");
        PUSH_SENSOR_INFO(oxVesselPressure, "OxVesselPressure");
        PUSH_SENSOR_INFO(oxFillingPressure, "OxFillingPressure");
        PUSH_SENSOR_INFO(n2Vessel1Pressure, "N2Vessel1Pressure");
        PUSH_SENSOR_INFO(n2Vessel2Pressure, "N2Vessel2Pressure");
        PUSH_SENSOR_INFO(n2FillingPressure, "N2FillingPressure");
        PUSH_SENSOR_INFO(oxTankBottomPressure, "OxTankBotPressure");
        PUSH_SENSOR_INFO(n2TankPressure, "N2TankPressure");
        PUSH_SENSOR_INFO(oxVesselWeight, "OxVesselWeight");
        PUSH_SENSOR_INFO(rocketWeight, "RocketWeight");
        PUSH_SENSOR_INFO(oxTankWeight, "OxTankWeight");

        return infos;
    }
    else
    {
        return {};
    }
}

void Sensors::internalAdcInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC1);

    internalAdc->enableChannel(InternalADC::CH9);
    internalAdc->enableChannel(InternalADC::CH11);
    internalAdc->enableChannel(
        Config::Sensors::InternalADC::BATTERY_VOLTAGE_CHANNEL);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    sdLogger.log(getInternalADCLastSample());
}

void Sensors::adc1Init()
{
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio = Config::Sensors::ADS131M08::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    // Configure all required channels
    config.channelsConfig[(int)Config::Sensors::ADC_1::OX_VESSEL_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::OX_FILLING_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::N2_VESSEL_1_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::N2_VESSEL_2_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::SERVO_CURRENT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::OX_VESSEL_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_32,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::ROCKET_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_32,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::N2_FILLING_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08_1(),
                                       sensors::ADS131_1::cs::getPin(),
                                       spiConfig, config);
}

void Sensors::adc1Callback() { sdLogger.log(ADC1Data{getADC1LastSample()}); }

void Sensors::adc2Init()
{
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio = Config::Sensors::ADS131M08::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    // Configure all required channels
    config.channelsConfig[(int)Config::Sensors::ADC_2::OX_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::N2_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};
    // Testing channels with filters, unnamed for now
    config.channelsConfig[3] = {.enabled = true,
                                .pga     = ADS131M08Defs::PGA::PGA_32,
                                .offset  = 0,
                                .gain    = 1.0};
    config.channelsConfig[5] = {.enabled = true,
                                .pga     = ADS131M08Defs::PGA::PGA_32,
                                .offset  = 0,
                                .gain    = 1.0};

    adc2 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08_2(),
                                       sensors::ADS131_2::cs::getPin(),
                                       spiConfig, config);
}

void Sensors::adc2Callback() { sdLogger.log(ADC2Data{getADC2LastSample()}); }

void Sensors::tc1Init()
{
    SPIBusConfig spiConfig = MAX31856::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    tc1 = std::make_unique<MAX31856>(
        getModule<Buses>()->getMAX31856_1(), sensors::MAX31856_1::cs::getPin(),
        spiConfig, MAX31856::ThermocoupleType::K_TYPE);
}

void Sensors::tc1Callback() { sdLogger.log(TC1Data{getTc1LastSample()}); }

void Sensors::oxVesselPressureInit()
{
    oxVesselPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::OX_VESSEL_PT_CHANNEL);
        },
        Config::Sensors::Trafag::OX_VESSEL_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_VESSEL_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::oxVesselPressureCallback()
{
    sdLogger.log(OxVesselPressureData{getOxVesselPressure()});
}

void Sensors::oxFillingPressureInit()
{
    oxFillingPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::OX_FILLING_PT_CHANNEL);
        },
        Config::Sensors::Trafag::OX_FILLING_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_FILLING_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::oxFillingPressureCallback()
{
    sdLogger.log(OxFillingPressureData{getOxFillingPressure()});
}

void Sensors::n2Vessel1PressureInit()
{
    n2Vessel1Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::N2_VESSEL_1_PT_CHANNEL);
        },
        Config::Sensors::Trafag::N2_VESSEL1_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::N2_VESSEL1_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::n2Vessel1PressureCallback()
{
    sdLogger.log(N2Vessel1PressureData{getN2Vessel1Pressure()});
}

void Sensors::n2Vessel2PressureInit()
{
    n2Vessel2Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::N2_VESSEL_2_PT_CHANNEL);
        },
        Config::Sensors::Trafag::N2_VESSEL2_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::N2_VESSEL2_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::n2Vessel2PressureCallback()
{
    sdLogger.log(N2Vessel2PressureData{getN2Vessel2Pressure()});
}

void Sensors::n2FillingPressureInit()
{
    n2FillingPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::N2_FILLING_PT_CHANNEL);
        },
        Config::Sensors::Trafag::N2_FILLING_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::N2_FILLING_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::n2FillingPressureCallback()
{
    sdLogger.log(N2FillingPressureData{getN2FillingPressure()});
}

void Sensors::oxTankBottomPressureInit()
{
    oxTankBottomPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::OX_TANK_PT_CHANNEL);
        },
        Config::Sensors::Trafag::OX_TANK_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_TANK_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::oxTankBottomPressureCallback()
{
    sdLogger.log(OxTankPressureData{oxTankBottomPressure->getLastSample()});
}

void Sensors::n2TankPressureInit()
{
    n2TankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::N2_TANK_PT_CHANNEL);
        },
        Config::Sensors::Trafag::N2_TANK_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::N2_TANK_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::n2TankPressureCallback()
{
    sdLogger.log(N2TankPressureData{n2TankPressure->getLastSample()});
}

void Sensors::oxVesselWeightInit()
{
    oxVesselWeight = std::make_unique<TwoPointAnalogLoadCell>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::OX_VESSEL_LC_CHANNEL);
        },
        Config::Sensors::LoadCell::VESSEL_P0_VOLTAGE,
        Config::Sensors::LoadCell::VESSEL_P0_MASS,
        Config::Sensors::LoadCell::VESSEL_P1_VOLTAGE,
        Config::Sensors::LoadCell::VESSEL_P1_MASS);
}

void Sensors::oxVesselWeightCallback()
{
    sdLogger.log(OxVesselWeightData{getOxVesselWeight()});
}

void Sensors::rocketWeightInit()
{
    rocketWeight = std::make_unique<TwoPointAnalogLoadCell>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(Config::Sensors::ADC_1::ROCKET_LC_CHANNEL);
        },
        Config::Sensors::LoadCell::ROCKET_P0_VOLTAGE,
        Config::Sensors::LoadCell::ROCKET_P0_MASS,
        Config::Sensors::LoadCell::ROCKET_P1_VOLTAGE,
        Config::Sensors::LoadCell::ROCKET_P1_MASS);
}

void Sensors::rocketWeightCallback()
{
    sdLogger.log(RocketWeightData{getRocketWeight()});
}

void Sensors::oxTankWeightInit()
{
    oxTankWeight = std::make_unique<TwoPointAnalogLoadCell>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(Config::Sensors::ADC_1::ROCKET_LC_CHANNEL);
        },
        Config::Sensors::LoadCell::ROCKET_P0_VOLTAGE,
        Config::Sensors::LoadCell::ROCKET_P0_MASS,
        Config::Sensors::LoadCell::ROCKET_P1_VOLTAGE,
        Config::Sensors::LoadCell::ROCKET_P1_MASS);
}

void Sensors::oxTankWeightCallback()
{
    sdLogger.log(OxTankWeightData{getOxTankWeight()});
}

bool Sensors::sensorManagerInit()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->sensors();

    SensorManager::SensorMap_t map;

    if (internalAdc)
    {
        SensorInfo info("InternalADC", Config::Sensors::InternalADC::PERIOD,
                        [this]() { internalAdcCallback(); });
        map.emplace(internalAdc.get(), info);
    }

    if (adc1)
    {
        SensorInfo info("ADS131M08_1", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { adc1Callback(); });
        map.emplace(std::make_pair(adc1.get(), info));
    }

    if (adc2)
    {
        SensorInfo info("ADS131M08_2", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { adc2Callback(); });
        map.emplace(std::make_pair(adc2.get(), info));
    }

    if (tc1)
    {
        SensorInfo info("MAX31856_1", Config::Sensors::MAX31856::PERIOD,
                        [this]() { tc1Callback(); });
        map.emplace(std::make_pair(tc1.get(), info));
    }

    if (oxVesselPressure)
    {
        SensorInfo info("OxVesselPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxVesselPressureCallback(); });
        map.emplace(std::make_pair(oxVesselPressure.get(), info));
    }

    if (oxFillingPressure)
    {
        SensorInfo info("OxFillingPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxFillingPressureCallback(); });
        map.emplace(std::make_pair(oxFillingPressure.get(), info));
    }

    if (n2Vessel1Pressure)
    {
        SensorInfo info("N2Vessel1Pressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { n2Vessel1PressureCallback(); });
        map.emplace(std::make_pair(n2Vessel1Pressure.get(), info));
    }

    if (n2Vessel2Pressure)
    {
        SensorInfo info("N2Vessel2Pressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { n2Vessel2PressureCallback(); });
        map.emplace(std::make_pair(n2Vessel2Pressure.get(), info));
    }

    if (n2FillingPressure)
    {
        SensorInfo info("N2FillingPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { n2FillingPressureCallback(); });
        map.emplace(std::make_pair(n2FillingPressure.get(), info));
    }

    if (oxTankBottomPressure)
    {
        SensorInfo info("OxTankBotPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxTankBottomPressureCallback(); });
        map.emplace(std::make_pair(oxTankBottomPressure.get(), info));
    }

    if (n2TankPressure)
    {
        SensorInfo info("N2TankPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { n2TankPressureCallback(); });
        map.emplace(std::make_pair(n2TankPressure.get(), info));
    }

    if (oxVesselWeight)
    {
        SensorInfo info("OxVesselWeight", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxVesselWeightCallback(); });
        map.emplace(std::make_pair(oxVesselWeight.get(), info));
    }

    if (rocketWeight)
    {
        SensorInfo info("RocketWeight", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { rocketWeightCallback(); });
        map.emplace(std::make_pair(rocketWeight.get(), info));
    }

    if (oxTankWeight)
    {
        SensorInfo info("OxTankWeight", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxTankWeightCallback(); });
        map.emplace(std::make_pair(oxTankWeight.get(), info));
    }

    manager = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}
