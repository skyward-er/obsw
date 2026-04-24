/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors:  Niccolò Betto, Pietro Bortolus
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

#include <RIGv3/Configs/SensorsConfig.h>
#include <RIGv3/Sensors/SensorsData.h>
#include <drivers/timer/TimestampTimer.h>
#include <interfaces-impl/hwmapping.h>

#include <chrono>

using namespace std::chrono;
using namespace Boardcore;
using namespace miosix;
using namespace RIGv3;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

    if (Config::Sensors::ADC_0::ENABLED)
    {
        adc0Init();
        przVessel1PressureInit();
        przVessel2PressureInit();
        przFillingPressureInit();
        oxVesselPressureInit();
        oxFillingPressureInit();
        oxVesselWeightInit();
    }
    if (Config::Sensors::ADC_1::ENABLED)
        adc1Init();

    if (Config::Sensors::ADC_2::ENABLED)
    {
        adc2Init();
        przTankPressureInit();
        oxRegOutPressureInit();
        fuelRegOutPressureInit();
        oxTankPressureInit();
        fuelTankPressureInit();
        igniterChamberPressureInit();
        mainChamberPressureInit();
    }
    if (Config::Sensors::ADC_3::ENABLED)
    {
        adc3Init();
        mainOxPositionInit();
        mainFuelPositionInit();
        oxRegPositionInit();
        fuelRegPositionInit();
        injOxPressureInit();
        injFuelPressureInit();
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

ADS131M08Data Sensors::getADC0LastSample()
{
    return adc0 ? adc0->getLastSample() : ADS131M08Data{};
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

PressureData Sensors::getPrzFillingPressure()
{
    return przFillingPressure ? przFillingPressure->getLastSample()
                              : PressureData{};
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

LoadCellData Sensors::getOxVesselWeight()
{
    return oxVesselWeight ? oxVesselWeight->getLastSample() : LoadCellData{};
}

ADS131M08Data Sensors::getADC1LastSample()
{
    return adc1 ? adc1->getLastSample() : ADS131M08Data{};
}

ADS131M08Data Sensors::getADC2LastSample()
{
    return adc2 ? adc2->getLastSample() : ADS131M08Data{};
}

PressureData Sensors::getPrzTankPressure()
{
    return przTankPressure ? przTankPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getOxRegOutPressure()
{
    return oxRegOutPressure ? oxRegOutPressure->getLastSample()
                            : PressureData{};
}

PressureData Sensors::getFuelRegOutPressure()
{
    return fuelRegOutPressure ? fuelRegOutPressure->getLastSample()
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

PressureData Sensors::getIgniterChamberPressure()
{
    return igniterChamberPressure ? igniterChamberPressure->getLastSample()
                                  : PressureData{};
}

PressureData Sensors::getMainChamberPressure()
{
    return mainChamberPressure ? mainChamberPressure->getLastSample()
                               : PressureData{};
}

ADS131M08Data Sensors::getADC3LastSample()
{
    return adc3 ? adc3->getLastSample() : ADS131M08Data{};
}

ServoPositionData Sensors::getMainOxPosition()
{
    return mainOxPosition ? mainOxPosition->getLastSample()
                          : ServoPositionData{};
}

ServoPositionData Sensors::getMainFuelPosition()
{
    return mainFuelPosition ? mainFuelPosition->getLastSample()
                            : ServoPositionData{};
}

ServoPositionData Sensors::getOxRegPosition()
{
    return oxRegPosition ? oxRegPosition->getLastSample() : ServoPositionData{};
}

ServoPositionData Sensors::getFuelRegPosition()
{
    return fuelRegPosition ? fuelRegPosition->getLastSample()
                           : ServoPositionData{};
}

PressureData Sensors::getInjOxPressure()
{
    return injOxPressure ? injOxPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getInjFuelPressure()
{
    return injFuelPressure ? injFuelPressure->getLastSample() : PressureData{};
}

CurrentData Sensors::getUmbilicalCurrent()
{
    // TODO: Implement umbilical current
    return {};
}

CurrentData Sensors::getServoCurrent()
{
    // TODO: Implement servo current
    return {};
}

VoltageData Sensors::getBatteryVoltage()
{
    // TODO: Implement battery voltage
    return {};
}

void Sensors::calibrate()
{
    using namespace Config::Sensors;

    constexpr auto ADC_CHANNEL_COUNT = ADS131M08Defs::CHANNELS_NUM;

    // One stat object per ADC channel
    Stats adcVoltageStats[4][ADC_CHANNEL_COUNT] = {};

    for (int i = 0; i < Trafag::CALIBRATE_SAMPLE_COUNT; i++)
    {
        auto adc0Sample = adc0->getLastSample();
        auto adc1Sample = adc1->getLastSample();
        auto adc2Sample = adc2->getLastSample();
        auto adc3Sample = adc3->getLastSample();

        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[0][j].add(adc0Sample.voltage[j]);
        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[1][j].add(adc1Sample.voltage[j]);
        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[2][j].add(adc2Sample.voltage[j]);
        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[3][j].add(adc3Sample.voltage[j]);

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
            adcVoltageStats[adcIndex][(size_t)ch].getStats().mean / minCurrent;

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

    using namespace Config::Sensors::ADC_0;
    applyShuntResistance(0, przVessel1Pressure, PRZ_VESSEL_1_PT_CHANNEL);
    applyShuntResistance(0, przVessel2Pressure, PRZ_VESSEL_2_PT_CHANNEL);
    applyShuntResistance(0, przFillingPressure, PRZ_FILLING_PT_CHANNEL);
    applyShuntResistance(0, oxVesselPressure, OX_VESSEL_PT_CHANNEL);
    applyShuntResistance(0, oxFillingPressure, OX_FILLING_PT_CHANNEL);

    using namespace Config::Sensors::ADC_2;
    applyShuntResistance(2, przTankPressure, PRZ_TANK_PT_CHANNEL);
    applyShuntResistance(2, oxRegOutPressure, OX_REG_OUT_PT_CHANNEL);
    applyShuntResistance(2, fuelRegOutPressure, FUEL_REG_OUT_PT_CHANNEL);
    applyShuntResistance(2, oxTankPressure, OX_TANK_PT_CHANNEL);
    applyShuntResistance(2, fuelTankPressure, FUEL_TANK_PT_CHANNEL);
    applyShuntResistance(2, igniterChamberPressure, IGNITER_CHAMBER_PT_CHANNEL);
    applyShuntResistance(2, mainChamberPressure, MAIN_CHAMBER_PT_CHANNEL);

    using namespace Config::Sensors::ADC_3;
    applyShuntResistance(3, injOxPressure, INJ_OX_PT_CHANNEL);
    applyShuntResistance(3, injFuelPressure, INJ_FUEL_PT_CHANNEL);

    calibrateEncoders();
}

void Sensors::calibrateLoadcells()
{
    Stats oxVesselStats;

    for (unsigned int i = 0;
         i < Config::Sensors::LoadCell::CALIBRATE_SAMPLE_COUNT; i++)
    {
        // Tank readings WITHOUT offsets
        oxVesselStats.add(oxVesselWeight->getLastSample().load);

        Thread::sleep(
            milliseconds{Config::Sensors::LoadCell::CALIBRATE_SAMPLE_PERIOD}
                .count());
    }

    oxVesselWeight->updateOffset(oxVesselStats.getStats().mean);
}

void Sensors::calibrateEncoders()
{
    mainFuelPosition->calibrate();
    mainOxPosition->calibrate();
    oxRegPosition->calibrate();
    fuelRegPosition->calibrate();
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

        PUSH_SENSOR_INFO(adc0, "ADS131M08_0");
        PUSH_SENSOR_INFO(przVessel1Pressure, "PrzVessel1Pressure");
        PUSH_SENSOR_INFO(przVessel2Pressure, "PrzVessel2Pressure");
        PUSH_SENSOR_INFO(przFillingPressure, "PrzFillingPressure");
        PUSH_SENSOR_INFO(oxVesselPressure, "OxVesselPressure");
        PUSH_SENSOR_INFO(oxFillingPressure, "OxFillingPressure");
        PUSH_SENSOR_INFO(oxVesselWeight, "OxVesselWeight");
        PUSH_SENSOR_INFO(adc1, "ADS131M08_1");
        PUSH_SENSOR_INFO(adc2, "ADS131M08_2");
        PUSH_SENSOR_INFO(przTankPressure, "PrzTankPressure");
        PUSH_SENSOR_INFO(oxRegOutPressure, "OxRegOutPressure");
        PUSH_SENSOR_INFO(fuelRegOutPressure, "FuelRegOutPressure");
        PUSH_SENSOR_INFO(oxTankPressure, "OxTankPressure");
        PUSH_SENSOR_INFO(fuelTankPressure, "FuelTankPressure");
        PUSH_SENSOR_INFO(igniterChamberPressure, "IgniterChamberPressure");
        PUSH_SENSOR_INFO(mainChamberPressure, "MainChamberPressure");
        PUSH_SENSOR_INFO(adc3, "ADS131M08_3");
        PUSH_SENSOR_INFO(mainOxPosition, "MainOxPosition");
        PUSH_SENSOR_INFO(mainFuelPosition, "MainFuelPosition");
        PUSH_SENSOR_INFO(oxRegPosition, "OxRegPosition");
        PUSH_SENSOR_INFO(fuelRegPosition, "FuelRegPosition");
        PUSH_SENSOR_INFO(injOxPressure, "InjOxPressure");
        PUSH_SENSOR_INFO(injFuelPressure, "InjFuelPressure");
        PUSH_SENSOR_INFO(internalAdc, "InternalADC");

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

    internalAdc->enableChannel(InternalADC::CH3);
    internalAdc->enableChannel(InternalADC::CH8);
    internalAdc->enableChannel(InternalADC::CH9);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    sdLogger.log(getInternalADCLastSample());
}

void Sensors::adc0Init()
{
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio = Config::Sensors::ADS131M08_SLOW::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08_SLOW::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    config.channelsConfig[(
        int)Config::Sensors::ADC_0::PRZ_VESSEL_1_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_0::PRZ_VESSEL_2_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_0::PRZ_FILLING_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_0::OX_VESSEL_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_0::OX_FILLING_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_0::OX_VESSEL_LC_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    adc0 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC0(),
                                       getModule<Buses>()->getADC0CsPin(),
                                       spiConfig, config);
}

void Sensors::adc0Callback() { sdLogger.log(ADC0Data{getADC0LastSample()}); }

void Sensors::przVessel1PressureInit()
{
    przVessel1Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC0LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_0::PRZ_VESSEL_1_PT_CHANNEL);
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
            auto sample = getADC0LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_0::PRZ_VESSEL_2_PT_CHANNEL);
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

void Sensors::przFillingPressureInit()
{
    przFillingPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC0LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_0::PRZ_FILLING_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::PRZ_FILLING_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::przFillingPressureCallback()
{
    sdLogger.log(PrzFillingPressureData{getPrzFillingPressure()});
}

void Sensors::oxVesselPressureInit()
{
    oxVesselPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC0LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_0::OX_VESSEL_PT_CHANNEL);
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

void Sensors::oxFillingPressureInit()
{
    oxFillingPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC0LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_0::OX_FILLING_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_FILLING_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::oxFillingPressureCallback()
{
    sdLogger.log(OxFillingPressureData{getOxFillingPressure()});
}

void Sensors::oxVesselWeightInit()
{
    oxVesselWeight = std::make_unique<TwoPointAnalogLoadCell>(
        [this]()
        {
            auto sample = getADC0LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_0::OX_VESSEL_LC_CHANNEL);
        },
        Config::Sensors::LoadCell::VESSEL_SCALE,
        Config::Sensors::LoadCell::VESSEL_OFFSET);
}

void Sensors::oxVesselWeightCallback()
{
    sdLogger.log(OxVesselWeightData{getOxVesselWeight()});
}

void Sensors::adc1Init()
{
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio = Config::Sensors::ADS131M08_SLOW::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08_SLOW::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC1(),
                                       getModule<Buses>()->getADC1CsPin(),
                                       spiConfig, config);
}
void Sensors::adc1Callback() { sdLogger.log(ADC1Data{getADC1LastSample()}); }

void Sensors::adc2Init()
{
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio = Config::Sensors::ADS131M08_FAST::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08_FAST::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    config.channelsConfig[(int)Config::Sensors::ADC_2::PRZ_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::OX_REG_OUT_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_2::FUEL_REG_OUT_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::OX_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::FUEL_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_2::IGNITER_CHAMBER_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_2::MAIN_CHAMBER_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    adc2 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC2(),
                                       getModule<Buses>()->getADC2CsPin(),
                                       spiConfig, config);
}
void Sensors::adc2Callback() { sdLogger.log(ADC2Data{getADC2LastSample()}); }

void Sensors::przTankPressureInit()
{
    przTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::PRZ_TANK_PT_CHANNEL);
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

void Sensors::oxRegOutPressureInit()
{
    oxRegOutPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::OX_REG_OUT_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::REG_OUT_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::oxRegOutPressureCallback()
{
    sdLogger.log(OxRegOutPressureData{getOxRegOutPressure()});
}

void Sensors::fuelRegOutPressureInit()
{
    fuelRegOutPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::FUEL_REG_OUT_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::REG_OUT_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::fuelRegOutPressureCallback()
{
    sdLogger.log(FuelRegOutPressureData{getFuelRegOutPressure()});
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
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::FUEL_TANK_PT_CHANNEL);
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

void Sensors::igniterChamberPressureInit()
{
    igniterChamberPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::IGNITER_CHAMBER_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::IGNITER_CHAMBER_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::igniterChamberPressureCallback()
{
    sdLogger.log(IgniterChamberPressureData{getIgniterChamberPressure()});
}

void Sensors::mainChamberPressureInit()
{
    mainChamberPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::MAIN_CHAMBER_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::MAIN_CHAMBER_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}
void Sensors::mainChamberPressureCallback()
{
    sdLogger.log(MainChamberPressureData{getMainChamberPressure()});
}

void Sensors::adc3Init()
{
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio = Config::Sensors::ADS131M08_FAST::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08_FAST::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    config.channelsConfig[(
        int)Config::Sensors::ADC_3::MAIN_OX_ENCODER_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_3::MAIN_FUEL_ENCODER_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_3::PRZ_OX_ENCODER_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_3::PRZ_FUEL_ENCODER_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};
    config.channelsConfig[(int)Config::Sensors::ADC_3::INJ_OX_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};
    config.channelsConfig[(int)Config::Sensors::ADC_3::INJ_FUEL_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    adc3 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC3(),
                                       getModule<Buses>()->getADC3CsPin(),
                                       spiConfig, config);
}
void Sensors::adc3Callback() { sdLogger.log(ADC3Data{getADC3LastSample()}); }

void Sensors::mainOxPositionInit()
{
    mainOxPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC3LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_3::MAIN_OX_ENCODER_CHANNEL);
        },
        Config::Sensors::Encoder::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Encoder::FULLSCALE_VOLTAGE,
        Config::Sensors::Encoder::SENSOR_RESISTANCE,
        Config::Sensors::Encoder::CURRENT_GAIN,
        Config::Sensors::Encoder::MAX_ANGLE);
}

void Sensors::mainOxPositionCallback()
{
    sdLogger.log(MainOxPositionData{getMainOxPosition()});
}

void Sensors::mainFuelPositionInit()
{
    mainFuelPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC3LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_3::MAIN_FUEL_ENCODER_CHANNEL);
        },
        Config::Sensors::Encoder::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Encoder::FULLSCALE_VOLTAGE,
        Config::Sensors::Encoder::SENSOR_RESISTANCE,
        Config::Sensors::Encoder::CURRENT_GAIN,
        Config::Sensors::Encoder::MAX_ANGLE);
}

void Sensors::mainFuelPositionCallback()
{
    sdLogger.log(MainFuelPositionData{getMainFuelPosition()});
}

void Sensors::oxRegPositionInit()
{
    oxRegPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC3LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_3::PRZ_OX_ENCODER_CHANNEL);
        },
        Config::Sensors::Encoder::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Encoder::FULLSCALE_VOLTAGE,
        Config::Sensors::Encoder::SENSOR_RESISTANCE,
        Config::Sensors::Encoder::CURRENT_GAIN,
        Config::Sensors::Encoder::MAX_ANGLE);
}

void Sensors::oxRegPositionCallback()
{
    sdLogger.log(OxRegPositionData{getOxRegPosition()});
}

void Sensors::fuelRegPositionInit()
{
    fuelRegPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC3LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_3::PRZ_FUEL_ENCODER_CHANNEL);
        },
        Config::Sensors::Encoder::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Encoder::FULLSCALE_VOLTAGE,
        Config::Sensors::Encoder::SENSOR_RESISTANCE,
        Config::Sensors::Encoder::CURRENT_GAIN,
        Config::Sensors::Encoder::MAX_ANGLE);
}

void Sensors::fuelRegPositionCallback()
{
    sdLogger.log(FuelRegPositionData{getFuelRegPosition()});
}

void Sensors::injOxPressureInit()
{
    injOxPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC3LastSample();
            return sample.getVoltage(Config::Sensors::ADC_3::INJ_OX_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::INJ_OX_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::injOxPressureCallback()
{
    sdLogger.log(InjOxPressureData{getInjOxPressure()});
}

void Sensors::injFuelPressureInit()
{
    injFuelPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC3LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_3::INJ_FUEL_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::INJ_FUEL_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::injFuelPressureCallback()
{
    sdLogger.log(InjFuelPressureData{getInjFuelPressure()});
}

bool Sensors::sensorManagerInit()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->sensors();

    SensorManager::SensorMap_t map;

    if (adc0)
    {
        SensorInfo info("ADC0", Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { adc0Callback(); });
        map.emplace(std::make_pair(adc0.get(), info));
    }

    if (przVessel1Pressure)
    {
        SensorInfo info("PrzVessel1Pressure",
                        Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { przVessel1PressureCallback(); });
        map.emplace(std::make_pair(przVessel1Pressure.get(), info));
    }

    if (przVessel2Pressure)
    {
        SensorInfo info("PrzVessel2Pressure",
                        Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { przVessel2PressureCallback(); });
        map.emplace(std::make_pair(przVessel2Pressure.get(), info));
    }

    if (przFillingPressure)
    {
        SensorInfo info("PrzFillingPressure",
                        Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { przFillingPressureCallback(); });
        map.emplace(std::make_pair(przFillingPressure.get(), info));
    }

    if (oxVesselPressure)
    {
        SensorInfo info("OxVesselPressure",
                        Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { oxVesselPressureCallback(); });
        map.emplace(std::make_pair(oxVesselPressure.get(), info));
    }

    if (oxFillingPressure)
    {
        SensorInfo info("OxFillingPressure",
                        Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { oxFillingPressureCallback(); });
        map.emplace(std::make_pair(oxFillingPressure.get(), info));
    }

    if (oxVesselWeight)
    {
        SensorInfo info("OxVesselWeight",
                        Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { oxVesselWeightCallback(); });
        map.emplace(std::make_pair(oxVesselWeight.get(), info));
    }

    if (adc1)
    {
        SensorInfo info("ADC1", Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { adc1Callback(); });
        map.emplace(std::make_pair(adc1.get(), info));
    }

    if (adc2)
    {
        SensorInfo info("ADC2", Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { adc2Callback(); });
        map.emplace(std::make_pair(adc2.get(), info));
    }

    if (przTankPressure)
    {
        SensorInfo info("PrzTankPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { przTankPressureCallback(); });
        map.emplace(std::make_pair(przTankPressure.get(), info));
    }

    if (oxRegOutPressure)
    {
        SensorInfo info("OxRegulatorPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { oxRegOutPressureCallback(); });
        map.emplace(std::make_pair(oxRegOutPressure.get(), info));
    }

    if (fuelRegOutPressure)
    {
        SensorInfo info("FuelRegulatorPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { fuelRegOutPressureCallback(); });
        map.emplace(std::make_pair(fuelRegOutPressure.get(), info));
    }

    if (oxTankPressure)
    {
        SensorInfo info("OxTankPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { oxTankPressureCallback(); });
        map.emplace(std::make_pair(oxTankPressure.get(), info));
    }

    if (fuelTankPressure)
    {
        SensorInfo info("FuelTankPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { fuelTankPressureCallback(); });
        map.emplace(std::make_pair(fuelTankPressure.get(), info));
    }

    if (igniterChamberPressure)
    {
        SensorInfo info("IgniterChamberPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { igniterChamberPressureCallback(); });
        map.emplace(std::make_pair(igniterChamberPressure.get(), info));
    }

    if (mainChamberPressure)
    {
        SensorInfo info("MainChamberPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { mainChamberPressureCallback(); });
        map.emplace(std::make_pair(mainChamberPressure.get(), info));
    }

    if (adc3)
    {
        SensorInfo info("ADC3", Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { adc3Callback(); });
        map.emplace(std::make_pair(adc3.get(), info));
    }

    if (mainOxPosition)
    {
        SensorInfo info("OxValvePosition",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { mainOxPositionCallback(); });
        map.emplace(std::make_pair(mainOxPosition.get(), info));
    }

    if (mainFuelPosition)
    {
        SensorInfo info("FuelValvePosition",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { mainFuelPositionCallback(); });
        map.emplace(std::make_pair(mainFuelPosition.get(), info));
    }

    if (oxRegPosition)
    {
        SensorInfo info("OxRegPosition",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { oxRegPositionCallback(); });
        map.emplace(std::make_pair(oxRegPosition.get(), info));
    }

    if (fuelRegPosition)
    {
        SensorInfo info("FuelRegPosition",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { fuelRegPositionCallback(); });
        map.emplace(std::make_pair(fuelRegPosition.get(), info));
    }

    if (injOxPressure)
    {
        SensorInfo info("InjOxPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { injOxPressureCallback(); });
        map.emplace(std::make_pair(injOxPressure.get(), info));
    }

    if (injFuelPressure)
    {
        SensorInfo info("InjFuelPressure",
                        Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { injFuelPressureCallback(); });
        map.emplace(std::make_pair(injFuelPressure.get(), info));
    }

    if (internalAdc)
    {
        SensorInfo info("InternalADC", Config::Sensors::InternalADC::PERIOD,
                        [this]() { internalAdcCallback(); });
        map.emplace(internalAdc.get(), info);
    }

    manager = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}
