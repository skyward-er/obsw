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

#include "Sensors.h"

#include <Motor/Actuators/Actuators.h>
#include <Motor/Buses.h>
#include <Motor/Configs/SensorsConfig.h>
#include <Motor/Sensors/SensorsData.h>
#include <interfaces-impl/hwmapping.h>

#include <chrono>

using namespace std::chrono;
using namespace Motor;
using namespace Boardcore;
using namespace miosix;

bool Sensors::start()
{
    if (Config::Sensors::ADC_1::ENABLED)
    {
        adc1Init();
        mainCCPressureInit();
        fuelTankPressureInit();
        oxTankPressureInit();
        przTankPressureInit();
        regulatorOutFuelPressureInit();
        regulatorOutOxPressureInit();
        ignCCPressureInit();
    }

    if (Config::Sensors::ADC_2::ENABLED)
    {
        adc2Init();
        mainFuelPositionInit();
        przFuelPositionInit();
        przOxPositionInit();
        ventingFuelPositionInit();
        ventingOxPositionInit();
    }

    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

    uint8_t taskId = getModule<BoardScheduler>()->sensors().addTask(
        [this] { checkOxTankOverpressure(); },
        Config::Sensors::OxTankOverpressure::CHECK_RATE);

    if (!taskId)
    {
        LOG_ERR(logger, "Failed to create OxTankOverpressure task");
        return false;
    }

    if (!postSensorCreationHook())
    {
        LOG_ERR(logger, "Failed to call postSensorCreationHook");
        return false;
    }

    if (!sensorManagerInit())
    {
        LOG_ERR(logger, "Failed to init SensorManager");
        return false;
    }

    return true;
}

void Sensors::calibrate()
{
    using namespace Config::Sensors;

    constexpr auto ADC_CHANNEL_COUNT = ADS131M08Defs::CHANNELS_NUM;

    // One stat object per ADC channel
    Stats adcVoltageStats[ADC_CHANNEL_COUNT] = {};

    for (int i = 0; i < Trafag::CALIBRATE_SAMPLE_COUNT; i++)
    {
        auto adcSample = adc1->getLastSample();

        for (int j = 0; j < ADC_CHANNEL_COUNT; j++)
            adcVoltageStats[j].add(adcSample.voltage[j]);

        Thread::sleep(
            milliseconds{Trafag::CALIBRATE_WAIT_BETWEEN_SAMPLES}.count());
    }

    // Applies the shunt resistance for the given channel to a trafag pressure
    // sensor, assuming the trafag is at atmospheric pressure reading
    // MIN_CURRENT (= 0 bar)
    auto applyShuntResistance = [&](auto& trafag, ADS131M08Defs::Channel ch)
    {
        constexpr float minCurrent = Trafag::MIN_CURRENT / 1000.0;  // [A]

        float resistance =
            adcVoltageStats[(size_t)ch].getStats().mean / minCurrent;

        // Ignore the calibrated shunt resistance if it's out of bounds
        if (resistance < Trafag::SHUNT_RESISTANCE_LOWER_BOUND ||
            resistance > Trafag::SHUNT_RESISTANCE_UPPER_BOUND)
        {
            resistance = trafag->getShuntResistance();
        }

        trafag->setShuntResistance(resistance);

#ifdef DEBUG
        fmt::print("\tChannel {}: {:.2f} Ohm\n", (int)ch, resistance);
#endif
    };

    using namespace Config::Sensors::ADC_1;

    applyShuntResistance(mainCCPressure, CC_PT_CHANNEL);
    applyShuntResistance(fuelTankPressure, FUEL_TANK_PT_CHANNEL);
    applyShuntResistance(przTankPressure, PRZ_TANK_PT_CHANNEL);
    applyShuntResistance(oxTankPressure, OX_TANK_PT_CHANNEL);
    applyShuntResistance(regOutFuelPressure, REGULATOR_OUT_FUEL_PT_CHANNEL);
    applyShuntResistance(regOutOxPressure, REGULATOR_OUT_OX_PT_CHANNEL);
    applyShuntResistance(ignCCPressure, IGNITER_PT_CHANNEL);
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

PressureData Sensors::getCCPressure()
{
    return mainCCPressure ? mainCCPressure->getLastSample() : PressureData{};
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

PressureData Sensors::getPrzTankPressure()
{
    return przTankPressure ? przTankPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getRegulatorOutFuelPressure()
{
    return regOutFuelPressure ? regOutFuelPressure->getLastSample()
                              : PressureData{};
}

PressureData Sensors::getRegulatorOutOxPressure()
{
    return regOutOxPressure ? regOutOxPressure->getLastSample()
                            : PressureData{};
}

PressureData Sensors::getIgniterPressure()
{
    return ignCCPressure ? ignCCPressure->getLastSample() : PressureData{};
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

ServoPositionData Sensors::getPrzOxPosition()
{
    return przOxPosition ? przOxPosition->getLastSample() : ServoPositionData{};
}

ServoPositionData Sensors::getPrzFuelPosition()
{
    return przFuelPosition ? przFuelPosition->getLastSample()
                           : ServoPositionData{};
}

ServoPositionData Sensors::getVentingOxPosition()
{
    return ventingOxPosition ? ventingOxPosition->getLastSample()
                             : ServoPositionData{};
}

ServoPositionData Sensors::getVentingFuelPosition()
{
    return ventingFuelPosition ? ventingFuelPosition->getLastSample()
                               : ServoPositionData{};
}

VoltageData Sensors::getBatteryVoltage()
{
    using namespace Config::Sensors::InternalADC;

    auto sample   = getInternalADCLastSample();
    float voltage = sample.voltage[(int)VBAT_CH] * VBAT_SCALE;
    return {sample.timestamp, voltage};
}

CurrentData Sensors::getCurrentConsumption()
{
    using namespace Config::Sensors::InternalADC;

    auto sample = getInternalADCLastSample();
    float current =
        sample.voltage[(int)CURRENT_CH] * CURRENT_SCALE + CURRENT_OFFSET;

    return {sample.timestamp, current};
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
        PUSH_SENSOR_INFO(mainCCPressure, "MainCCPressure");
        PUSH_SENSOR_INFO(oxTankPressure, "OxTankPressure");
        PUSH_SENSOR_INFO(fuelTankPressure, "FuelTankPressure");
        PUSH_SENSOR_INFO(przTankPressure, "PrzTankPressure");
        PUSH_SENSOR_INFO(regOutOxPressure, "RegulatorOutOxPressure");
        PUSH_SENSOR_INFO(regOutFuelPressure, "RegulatorOutFuelPressure");
        PUSH_SENSOR_INFO(ignCCPressure, "IgnCCPressure");
        PUSH_SENSOR_INFO(mainOxPosition, "MainOxPosition");
        PUSH_SENSOR_INFO(mainOxPosition, "MainOxPosition");
        PUSH_SENSOR_INFO(przFuelPosition, "PrzFuelPosition");
        PUSH_SENSOR_INFO(przOxPosition, "PrzOxPosition");
        PUSH_SENSOR_INFO(ventingFuelPosition, "VentingFuelPosition");
        PUSH_SENSOR_INFO(ventingOxPosition, "VentingOxPosition");

        return infos;
    }
    else
    {
        return {};
    }
}

TaskScheduler& Sensors::getSensorsScheduler()
{
    return getModule<BoardScheduler>()->sensors();
}

void Sensors::adc1Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio     = Config::Sensors::ADC_1::OSR;
    config.globalChopModeEnabled = Config::Sensors::ADC_1::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    // Configure all required channels
    config.channelsConfig[(int)Config::Sensors::ADC_1::CC_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::FUEL_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::PRZ_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::OX_TANK_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_1::REGULATOR_OUT_FUEL_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_1::REGULATOR_OUT_OX_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_1::IGNITER_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC1(),
                                       sensors::ADC_1::cs::getPin(), spiConfig,
                                       config);
}

void Sensors::adc1Callback() { sdLogger.log(getADC1LastSample()); }

void Sensors::adc2Init()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config = {};
    // Setup global configurations
    config.oversamplingRatio     = Config::Sensors::ADC_2::OSR;
    config.globalChopModeEnabled = Config::Sensors::ADC_2::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
        channel.enabled = false;

    // Configure all required channels
    config.channelsConfig[(int)Config::Sensors::ADC_2::FUEL_MAIN_EN_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::PRZ_OX_EN_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::PRZ_FUEL_EN_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADC_2::OX_VENTING_EN_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADC_2::FUEL_VENTING_EN_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    adc2 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC2(),
                                       sensors::ADC_1::cs::getPin(), spiConfig,
                                       config);
}

void Sensors::adc2Callback() { sdLogger.log(getADC2LastSample()); }

void Sensors::internalAdcInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(Config::Sensors::InternalADC::CURRENT_CH);
    internalAdc->enableChannel(Config::Sensors::InternalADC::VBAT_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    sdLogger.log(getInternalADCLastSample());
}

void Sensors::regulatorOutFuelPressureInit()
{
    regOutFuelPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::REGULATOR_OUT_FUEL_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::REGULATOR_OUT_FUEL_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::regulatorOutFuelPressureCallback()
{
    sdLogger.log(RegulatorOutFuelPressureData{getRegulatorOutFuelPressure()});
}

void Sensors::regulatorOutOxPressureInit()
{
    regOutFuelPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::REGULATOR_OUT_OX_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::REGULATOR_OUT_OX_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::regulatorOutOxPressureCallback()
{
    sdLogger.log(RegulatorOutOxPressureData{getRegulatorOutOxPressure()});
}

void Sensors::oxTankPressureInit()
{
    oxTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::OX_TANK_PT_CHANNEL);
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

void Sensors::fuelTankPressureInit()
{
    przTankPressure = std::make_unique<TrafagPressureSensor>(
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

void Sensors::ignCCPressureInit()
{
    ignCCPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_1::IGNITER_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::IGNITER_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::ignCCPressureCallback()
{
    sdLogger.log(IgniterPressureData{getIgniterPressure()});
}

void Sensors::mainCCPressureInit()
{
    mainCCPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADC1LastSample();
            return sample.getVoltage(Config::Sensors::ADC_1::CC_PT_CHANNEL);
        },
        Config::Sensors::Trafag::DEFAULT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::CC_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::mainCCPressureCallback()
{
    sdLogger.log(CCPressureData{getCCPressure()});
}

void Sensors::mainOxPositionInit()
{
    mainOxPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::OX_MAIN_EN_CHANNEL);
        });
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
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::FUEL_MAIN_EN_CHANNEL);
        });
}

void Sensors::mainFuelPositionCallback()
{
    sdLogger.log(MainFuelPositionData{getMainFuelPosition()});
}

void Sensors::przOxPositionInit()
{
    przOxPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(Config::Sensors::ADC_2::PRZ_OX_EN_CHANNEL);
        });
}

void Sensors::przOxPositionCallback()
{
    sdLogger.log(PrzOxPositionData{getPrzOxPosition()});
}

void Sensors::przFuelPositionInit()
{
    przFuelPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::PRZ_FUEL_EN_CHANNEL);
        });
}

void Sensors::przFuelPositionCallback()
{
    sdLogger.log(PrzFuelPositionData{getPrzFuelPosition()});
}

void Sensors::ventingOxPositionInit()
{
    ventingOxPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::OX_VENTING_EN_CHANNEL);
        });
}

void Sensors::ventingOxPositionCallback()
{
    sdLogger.log(VentingOxPositionData{getVentingOxPosition()});
}

void Sensors::ventingFuelPositionInit()
{
    ventingFuelPosition = std::make_unique<AnalogEncoder>(
        [this]()
        {
            auto sample = getADC2LastSample();
            return sample.getVoltage(
                Config::Sensors::ADC_2::FUEL_VENTING_EN_CHANNEL);
        });
}

void Sensors::ventingFuelPositionCallback()
{
    sdLogger.log(VentingFuelPositionData{getVentingFuelPosition()});
}

bool Sensors::sensorManagerInit()
{
    SensorManager::SensorMap_t map;

    if (adc1)
    {
        SensorInfo adc1Info{"ADC1", Config::Sensors::ADC_1::RATE,
                            [this]() { adc1Callback(); }};
        map.emplace(adc1.get(), adc1Info);
    }

    if (adc2)
    {
        SensorInfo adc2Info{"ADC_2", Config::Sensors::ADC_2::RATE,
                            [this]() { adc2Callback(); }};
        map.emplace(adc2.get(), adc2Info);
    }

    if (internalAdc)
    {
        SensorInfo internalAdcInfo{"InternalADC",
                                   Config::Sensors::InternalADC::RATE,
                                   [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), internalAdcInfo);
    }

    if (regOutFuelPressure)
    {
        SensorInfo info{"RegulatorOutFuelPressure",
                        Config::Sensors::ADC_1::RATE,
                        [this]() { regulatorOutFuelPressureCallback(); }};
        map.emplace(std::make_pair(regOutFuelPressure.get(), info));
    }

    if (regOutOxPressure)
    {
        SensorInfo info{"RegulatorOutOxPressure", Config::Sensors::ADC_1::RATE,
                        [this]() { regulatorOutOxPressureCallback(); }};
        map.emplace(std::make_pair(regOutOxPressure.get(), info));
    }

    if (oxTankPressure)
    {
        SensorInfo info{"OxBottomTank0Pressure", Config::Sensors::ADC_1::RATE,
                        [this]() { oxTankPressureCallback(); }};
        map.emplace(std::make_pair(oxTankPressure.get(), info));
    }

    if (fuelTankPressure)
    {
        SensorInfo info{"FuelTankPressure", Config::Sensors::ADC_1::RATE,
                        [this]() { fuelTankPressureCallback(); }};
        map.emplace(std::make_pair(fuelTankPressure.get(), info));
    }

    if (mainCCPressure)
    {
        SensorInfo info{"MainCCPressure", Config::Sensors::ADC_1::RATE,
                        [this]() { mainCCPressureCallback(); }};
        map.emplace(std::make_pair(mainCCPressure.get(), info));
    }

    if (ignCCPressure)
    {
        SensorInfo info{"IgnCCPressure", Config::Sensors::ADC_1::RATE,
                        [this]() { ignCCPressureCallback(); }};
        map.emplace(std::make_pair(ignCCPressure.get(), info));
    }

    if (mainFuelPosition)
    {
        SensorInfo info{"MainFuelPosition", Config::Sensors::ADC_2::RATE,
                        [this]() { mainFuelPositionCallback(); }};
        map.emplace(std::make_pair(mainFuelPosition.get(), info));
    }

    if (przOxPosition)
    {
        SensorInfo info{"PrzOxPosition", Config::Sensors::ADC_2::RATE,
                        [this]() { przOxPositionCallback(); }};
        map.emplace(std::make_pair(przOxPosition.get(), info));
    }

    if (przFuelPosition)
    {
        SensorInfo info{"PrzFuelPosition", Config::Sensors::ADC_2::RATE,
                        [this]() { przFuelPositionCallback(); }};
        map.emplace(std::make_pair(przFuelPosition.get(), info));
    }

    if (ventingOxPosition)
    {
        SensorInfo info{"VentingOxPosition", Config::Sensors::ADC_2::RATE,
                        [this]() { ventingOxPositionCallback(); }};
        map.emplace(std::make_pair(ventingOxPosition.get(), info));
    }

    if (ventingFuelPosition)
    {
        SensorInfo info{"VentingFuelPosition", Config::Sensors::ADC_2::RATE,
                        [this]() { ventingFuelPositionCallback(); }};
        map.emplace(std::make_pair(ventingFuelPosition.get(), info));
    }

    manager = std::make_unique<SensorManager>(map, &getSensorsScheduler());
    return manager->start();
}

