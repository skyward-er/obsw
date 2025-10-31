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
#include <common/canbus/MotorStatus.h>
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
        regulatorPressureInit();
        przVessel1PressureInit();
        fuelTankPressureInit();
        oxVesselWeightInit();
        przTankPressureInit();

        rocketWeightInit();
        oxTankWeightInit();
    }

    if (Config::Sensors::ADC_2::ENABLED)
    {
        adc2Init();
        oxTankPressureInit();
        oxValvePositionInit();
        fuelValvePositionInit();
        przVessel2PressureInit();
    }

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

PressureData Sensors::getOxVesselPressure()
{
    return oxVesselPressure ? oxVesselPressure->getLastSample()
                            : PressureData{};
}

PressureData Sensors::getPrzVessel1Pressure()
{
    return przVessel1Pressure ? przVessel1Pressure->getLastSample()
                              : PressureData{};
}

PressureData Sensors::getPrzVessel2Pressure()
{
    return przVessel2Pressure ? przVessel2Pressure->getLastSample()
                              : PressureData{};
}

PressureData Sensors::getPrzTankPressure()
{
    return przTankPressure ? przTankPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getRegulatorPressure()
{
    return regulatorPressure ? regulatorPressure->getLastSample()
                             : PressureData{};
}

PressureData Sensors::getOxTankPressure()
{
    return oxTankPressure ? oxTankPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getFuelTankPressure()
{
    return fuelTankPressure ? fuelTankPressure->getLastSample()
                            : PressureData{};
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

ServoPositionData Sensors::getOxValvePosition()
{
    return oxValvePosition ? oxValvePosition->getLastSample()
                           : ServoPositionData{};
}

ServoPositionData Sensors::getFuelValvePosition()
{
    return fuelValvePosition ? fuelValvePosition->getLastSample()
                             : ServoPositionData{};
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

PressureData Sensors::getOxTankBottomPressureDirectOrCan()
{
    /* auto motor = getModule<Common::MotorStatus>();

    if (motor->detected())
    {
        auto data = motor->lockData();
        return data->oxTankBottom0Pressure;
    }
    else
    {
        return getOxTankBottomPressure();
    } */

    PressureData pressure = getOxTankPressure();
    return pressure;
}

void Sensors::calibrate()
{
    using namespace Config::Sensors;

    constexpr auto ADC_CHANNEL_COUNT = ADS131M08Defs::CHANNELS_NUM;

    // One stat object per ADC channel
    Stats adcVoltageStats[2][ADC_CHANNEL_COUNT] = {};

    for (int i = 0; i < Trafag::CALIBRATE_SAMPLE_COUNT; i++)
    {
        auto adc1Sample = adc1->getLastSample();
        auto adc2Sample = adc2->getLastSample();

        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[0][j].add(adc1Sample.voltage[j]);
        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[1][j].add(adc2Sample.voltage[j]);

        Thread::sleep(
            milliseconds{Trafag::CALIBRATE_WAIT_BETWEEN_SAMPLES}.count());
    }

    // Applies the shunt resistance for the given channel to a trafag pressure
    // sensor, assuming the trafag is at atmospheric pressure reading
    // MIN_CURRENT (= 0 bar)
    auto applyShuntResistance =
        [&](int adcIndex, auto& trafag, ADS131M08Defs::Channel ch)
    {
        constexpr float minCurrent = Trafag::MIN_CURRENT / 1000.0;  // [A]

        float resistance =
            adcVoltageStats[adcIndex - 1][(size_t)ch].getStats().mean /
            minCurrent;

        // Ignore the calibrated shunt resistance if it's out of bounds
        if (resistance < Trafag::SHUNT_RESISTANCE_LOWER_BOUND ||
            resistance > Trafag::SHUNT_RESISTANCE_UPPER_BOUND)
        {
            resistance = trafag->getShuntResistance();
        }

        trafag->setShuntResistance(resistance);

#ifdef DEBUG
        fmt::print("\tADC {} - Channel {}: {:.2f} Ohm\n", adcIndex, (int)ch,
                   resistance);
#endif
    };

    using namespace Config::Sensors::ADC_1;
    applyShuntResistance(1, oxVesselPressure, OX_VESSEL_PT_CHANNEL);
    applyShuntResistance(1, regulatorPressure, REGULATOR_PT_CHANNEL);
    applyShuntResistance(1, przVessel1Pressure, PRZ_VESSEL_1_PT_CHANNEL);
    applyShuntResistance(1, fuelTankPressure, FUEL_TANK_PT_CHANNEL);
    applyShuntResistance(1, przTankPressure, FUEL_TANK_PT_CHANNEL);

    using namespace Config::Sensors::ADC_2;
    applyShuntResistance(2, oxTankPressure, OX_TANK_PT_CHANNEL);
    applyShuntResistance(2, przVessel2Pressure, PRZ_VESSEL_2_PT_CHANNEL);

    fuelValvePosition->calibrate();
    oxValvePosition->calibrate();
}

void Sensors::calibrateLoadcells()
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

void Sensors::calibrateEncoders()
{
    fuelValvePosition->calibrate();
    oxValvePosition->calibrate();
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
        PUSH_SENSOR_INFO(internalAdc, "InternalADC");
        PUSH_SENSOR_INFO(oxVesselPressure, "OxVesselPressure");
        PUSH_SENSOR_INFO(przVessel1Pressure, "PrzVessel1Pressure");
        PUSH_SENSOR_INFO(przVessel2Pressure, "PrzVessel2Pressure");
        PUSH_SENSOR_INFO(przTankPressure, "PrzTankPressure");
        PUSH_SENSOR_INFO(regulatorPressure, "RegulatorPressure");
        PUSH_SENSOR_INFO(oxTankPressure, "OxTankPressure");
        PUSH_SENSOR_INFO(fuelTankPressure, "FuelTankPressure");
        PUSH_SENSOR_INFO(oxVesselWeight, "OxVesselWeight");
        PUSH_SENSOR_INFO(oxValvePosition, "OxValvePosition");
        PUSH_SENSOR_INFO(fuelValvePosition, "FuelValvePosition");

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

    config.channelsConfig[(int)Config::Sensors::ADC_1::REGULATOR_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_1::PRZ_VESSEL_1_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::FUEL_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::OX_VESSEL_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_32,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::PRZ_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    /* config.channelsConfig[(int)Config::Sensors::ADC_1::SERVO_CURRENT_CHANNEL]
=
{.enabled = true,
     .pga     = ADS131M08Defs::PGA::PGA_1,
     .offset  = 0,
     .gain    = 1.0};

     config.channelsConfig[(int)Config::Sensors::ADC_1::ROCKET_LC_CHANNEL] = {
    .enabled = true,
    .pga     = ADS131M08Defs::PGA::PGA_32,
    .offset  = 0,
    .gain    = 1.0}; */

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

    config.channelsConfig[(int)Config::Sensors::ADC_2::OX_VALVE_ENC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::FUEL_VALVE_ENC_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_2::PRZ_VESSEL_2_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    adc2 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08_2(),
                                       sensors::ADS131_2::cs::getPin(),
                                       spiConfig, config);
}

void Sensors::adc2Callback() { sdLogger.log(ADC2Data{getADC2LastSample()}); }

void Sensors::oxVesselPressureInit()
{
    oxVesselPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::OX_VESSEL_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_VESSEL_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::oxVesselPressureCallback()
{
    sdLogger.log(OxVesselPressureData{getOxVesselPressure()});
}

void Sensors::przVessel1PressureInit()
{
    przVessel1Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::PRZ_VESSEL_1_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::PRZ_VESSEL_1_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::przVessel1PressureCallback()
{
    sdLogger.log(PrzVessel1PressureData{getPrzVessel1Pressure()});
}

void Sensors::przVessel2PressureInit()
{
    przVessel2Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::PRZ_VESSEL_2_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::PRZ_VESSEL_2_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::przVessel2PressureCallback()
{
    sdLogger.log(PrzVessel2PressureData{getPrzVessel2Pressure()});
}

void Sensors::przTankPressureInit()
{
    przTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::PRZ_TANK_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::PRZ_TANK_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::przTankPressureCallback()
{
    sdLogger.log(PrzTankPressureData{getPrzTankPressure()});
}

void Sensors::regulatorPressureInit()
{
    regulatorPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::REGULATOR_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::REGULATOR_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::regulatorPressureCallback()
{
    sdLogger.log(RegulatorPressureData{getRegulatorPressure()});
}

void Sensors::oxTankPressureInit()
{
    oxTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::OX_TANK_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_TANK_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::oxTankPressureCallback()
{
    sdLogger.log(OxTankPressureData{getOxTankPressure()});
}

void Sensors::fuelTankPressureInit()
{
    fuelTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::FUEL_TANK_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::FUEL_TANK_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::fuelTankPressureCallback()
{
    sdLogger.log(FuelTankPressureData{getFuelTankPressure()});
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

void Sensors::oxValvePositionInit()
{
    oxValvePosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::OX_VALVE_ENC_CHANNEL);
        },
        Config::Sensors::Encoder::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Encoder::FULLSCALE_VOLTAGE,
        Config::Sensors::Encoder::SENSOR_RESISTANCE,
        Config::Sensors::Encoder::CURRENT_GAIN,
        Config::Sensors::Encoder::MAX_ANGLE);
}

void Sensors::oxValvePositionCallback()
{
    sdLogger.log(OxValvePositionData{getOxValvePosition()});
}

void Sensors::fuelValvePositionInit()
{
    fuelValvePosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::FUEL_VALVE_ENC_CHANNEL);
        },
        Config::Sensors::Encoder::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Encoder::FULLSCALE_VOLTAGE,
        Config::Sensors::Encoder::SENSOR_RESISTANCE,
        Config::Sensors::Encoder::CURRENT_GAIN,
        Config::Sensors::Encoder::MAX_ANGLE);
}

void Sensors::fuelValvePositionCallback()
{
    sdLogger.log(FuelValvePositionData{getFuelValvePosition()});
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

    if (oxVesselPressure)
    {
        SensorInfo info("OxVesselPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxVesselPressureCallback(); });
        map.emplace(std::make_pair(oxVesselPressure.get(), info));
    }

    if (przVessel1Pressure)
    {
        SensorInfo info("PrzVessel1Pressure",
                        Config::Sensors::ADS131M08::PERIOD,
                        [this]() { przVessel1PressureCallback(); });
        map.emplace(std::make_pair(przVessel1Pressure.get(), info));
    }

    if (przVessel2Pressure)
    {
        SensorInfo info("PrzVessel2Pressure",
                        Config::Sensors::ADS131M08::PERIOD,
                        [this]() { przVessel2PressureCallback(); });
        map.emplace(std::make_pair(przVessel2Pressure.get(), info));
    }

    if (przTankPressure)
    {
        SensorInfo info("PrzTankPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { przTankPressureCallback(); });
        map.emplace(std::make_pair(przTankPressure.get(), info));
    }

    if (regulatorPressure)
    {
        SensorInfo info("RegulatorPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { regulatorPressureCallback(); });
        map.emplace(std::make_pair(regulatorPressure.get(), info));
    }

    if (oxTankPressure)
    {
        SensorInfo info("OxTankPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxTankPressureCallback(); });
        map.emplace(std::make_pair(oxTankPressure.get(), info));
    }

    if (fuelTankPressure)
    {
        SensorInfo info("FuelTankPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { fuelTankPressureCallback(); });
        map.emplace(std::make_pair(fuelTankPressure.get(), info));
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

    if (oxValvePosition)
    {
        SensorInfo info("OxValvePosition", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { oxValvePositionCallback(); });
        map.emplace(std::make_pair(oxValvePosition.get(), info));
    }

    if (fuelValvePosition)
    {
        SensorInfo info("FuelValvePosition", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { fuelValvePositionCallback(); });
        map.emplace(std::make_pair(fuelValvePosition.get(), info));
    }

    manager = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}
