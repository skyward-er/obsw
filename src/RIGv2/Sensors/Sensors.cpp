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
#include <drivers/timer/TimestampTimer.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace miosix;
using namespace RIGv2;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

    if (Config::Sensors::ADS131M08::ENABLED)
    {
        adc1Init();
        vesselPressureInit();
        fillingPressureInit();
        topTankPressureInit();
        bottomTankPressureInit();
        vesselWeightInit();
        tankWeightInit();
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

MAX31856Data Sensors::getTc1LastSample()
{
    return tc1 ? tc1->getLastSample() : MAX31856Data{};
}

PressureData Sensors::getVesselPressLastSample()
{
    return vesselPressure ? vesselPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getFillingPressLastSample()
{
    return fillingPressure ? fillingPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getTopTankPressLastSample()
{
    if (useCanData)
    {
        return getCanTopTankPressLastSample();
    }
    else
    {
        return topTankPressure ? topTankPressure->getLastSample()
                               : PressureData{};
    }
}

PressureData Sensors::getBottomTankPressLastSample()
{
    if (useCanData)
    {
        return getCanBottomTankPressLastSample();
    }
    else
    {
        return bottomTankPressure ? bottomTankPressure->getLastSample()
                                  : PressureData{};
    }
}

PressureData Sensors::getCCPressLastSample()
{
    if (useCanData)
        return getCanCCPressLastSample();
    else
        return PressureData{};
}

TemperatureData Sensors::getTankTempLastSample()
{
    if (useCanData)
        return getCanTankTempLastSample();
    else
        return getTc1LastSample();
}

LoadCellData Sensors::getVesselWeightLastSample()
{
    if (vesselWeight)
        return vesselWeight->getLastSample();
    else
        return {};
}

LoadCellData Sensors::getTankWeightLastSample()
{
    if (tankWeight)
        return tankWeight->getLastSample();
    else
        return {};
}

CurrentData Sensors::getUmbilicalCurrentLastSample()
{
    // TODO: Implement this
    return {};
}

CurrentData Sensors::getServoCurrentLastSample()
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

VoltageData Sensors::getBatteryVoltageLastSample()
{
    auto sample = getInternalADCLastSample();

    float voltage =
        sample.voltage[(
            int)Config::Sensors::InternalADC::BATTERY_VOLTAGE_CHANNEL] *
        Config::Sensors::InternalADC::BATTERY_VOLTAGE_SCALE;
    return {sample.timestamp, voltage};
}

VoltageData Sensors::getMotorBatteryVoltageLastSample()
{
    if (useCanData)
        return getCanMotorBatteryVoltageLastSample();
    else
        return VoltageData{};
}

PressureData Sensors::getCanTopTankPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canTopTankPressure;
}

PressureData Sensors::getCanBottomTankPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canBottomTankPressure;
}

PressureData Sensors::getCanCCPressLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canCCPressure;
}

TemperatureData Sensors::getCanTankTempLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canTankTemperature;
}

VoltageData Sensors::getCanMotorBatteryVoltageLastSample()
{
    Lock<FastMutex> lock{canMutex};
    return canMotorBatteryVoltage;
}

void Sensors::setCanTopTankPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTopTankPressure = data;
}

void Sensors::setCanBottomTankPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canBottomTankPressure = data;
}

void Sensors::setCanCCPress(PressureData data)
{
    Lock<FastMutex> lock{canMutex};
    canCCPressure = data;
}

void Sensors::setCanTankTemp(TemperatureData data)
{
    Lock<FastMutex> lock{canMutex};
    canTankTemperature = data;
}

void Sensors::setCanMotorBatteryVoltage(VoltageData data)
{
    Lock<FastMutex> lock{canMutex};
    canMotorBatteryVoltage = data;
}

void Sensors::switchToCanSensors() { useCanData = true; }

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
        std::vector<SensorInfo> infos{};

        if (vesselWeight)
            infos.push_back(manager->getSensorInfo(vesselWeight.get()));

        if (tankWeight)
            infos.push_back(manager->getSensorInfo(tankWeight.get()));

        if (vesselPressure)
            infos.push_back(manager->getSensorInfo(vesselPressure.get()));

        if (fillingPressure)
            infos.push_back(manager->getSensorInfo(fillingPressure.get()));

        if (topTankPressure)
            infos.push_back(manager->getSensorInfo(topTankPressure.get()));

        if (bottomTankPressure)
            infos.push_back(manager->getSensorInfo(bottomTankPressure.get()));

        if (internalAdc)
            infos.push_back(manager->getSensorInfo(internalAdc.get()));

        if (adc1)
            infos.push_back(manager->getSensorInfo(adc1.get()));

        if (tc1)
            infos.push_back(manager->getSensorInfo(tc1.get()));

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
}

void Sensors::adc1Callback() { sdLogger.log(ADC1Data{getADC1LastSample()}); }

void Sensors::tc1Init()
{
    SPIBusConfig spiConfig = MAX31856::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    tc1 = std::make_unique<MAX31856>(
        getModule<Buses>()->getMAX31856_1(), sensors::MAX31856_1::cs::getPin(),
        spiConfig, MAX31856::ThermocoupleType::K_TYPE);
}

void Sensors::tc1Callback() { sdLogger.log(TC1Data{getTc1LastSample()}); }

void Sensors::vesselPressureInit()
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
}

void Sensors::vesselPressureCallback()
{
    sdLogger.log(VesselPressureData{getVesselPressLastSample()});
}

void Sensors::fillingPressureInit()
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
}

void Sensors::fillingPressureCallback()
{
    sdLogger.log(FillingPressureData{getFillingPressLastSample()});
}

void Sensors::topTankPressureInit()
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
}

void Sensors::topTankPressureCallback()
{
    sdLogger.log(TopTankPressureData{topTankPressure->getLastSample()});
}

void Sensors::bottomTankPressureInit()
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
}

void Sensors::bottomTankPressureCallback()
{
    sdLogger.log(BottomTankPressureData{bottomTankPressure->getLastSample()});
}

void Sensors::vesselWeightInit()
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
}

void Sensors::vesselWeightCallback()
{
    sdLogger.log(VesselWeightData{getVesselWeightLastSample()});
}

void Sensors::tankWeightInit()
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
}

void Sensors::tankWeightCallback()
{
    sdLogger.log(TankWeightData{getTankWeightLastSample()});
}

bool Sensors::sensorManagerInit()
{
    TaskScheduler& scheduler =
        getModule<BoardScheduler>()->getSensorsScheduler();

    SensorManager::SensorMap_t map;

    if (internalAdc)
    {
        SensorInfo info("InternalAdc", Config::Sensors::InternalADC::PERIOD,
                        [this]() { internalAdcCallback(); });
        map.emplace(internalAdc.get(), info);
    }

    if (adc1)
    {
        SensorInfo info("ADS131M08_1", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { adc1Callback(); });
        map.emplace(std::make_pair(adc1.get(), info));
    }

    if (tc1)
    {
        SensorInfo info("MAX31856_1", Config::Sensors::MAX31856::PERIOD,
                        [this]() { tc1Callback(); });
        map.emplace(std::make_pair(tc1.get(), info));
    }

    if (vesselPressure)
    {
        SensorInfo info("VesselPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { vesselPressureCallback(); });
        map.emplace(std::make_pair(vesselPressure.get(), info));
    }

    if (fillingPressure)
    {
        SensorInfo info("FillingPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { fillingPressureCallback(); });
        map.emplace(std::make_pair(fillingPressure.get(), info));
    }

    if (topTankPressure)
    {
        SensorInfo info("TopTankPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { topTankPressureCallback(); });
        map.emplace(std::make_pair(topTankPressure.get(), info));
    }

    if (bottomTankPressure)
    {
        SensorInfo info("BottomTankPressure",
                        Config::Sensors::ADS131M08::PERIOD,
                        [this]() { bottomTankPressureCallback(); });
        map.emplace(std::make_pair(bottomTankPressure.get(), info));
    }

    if (vesselWeight)
    {
        SensorInfo info("VesselWeight", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { vesselWeightCallback(); });
        map.emplace(std::make_pair(vesselWeight.get(), info));
    }

    if (tankWeight)
    {
        SensorInfo info("TankWeight", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { tankWeightCallback(); });
        map.emplace(std::make_pair(tankWeight.get(), info));
    }

    manager = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}
