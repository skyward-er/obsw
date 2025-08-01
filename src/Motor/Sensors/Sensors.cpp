/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Fabrizio Monti, Niccolò Betto
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
    if (Config::Sensors::LPS22DF::ENABLED)
        lps22dfInit();

    if (Config::Sensors::H3LIS331DL::ENABLED)
        h3lis331dlInit();

    if (Config::Sensors::LIS2MDL::ENABLED)
        lis2mdlInit();

    if (Config::Sensors::LSM6DSRX::ENABLED)
        lsm6dsrxInit();

    if (Config::Sensors::ADS131M08::ENABLED)
    {
        ads131m08Init();
        regulatorOutPressureInit();
        oxTankTopPressureInit();
        oxTankBottomPressureInit();
        n2TankPressureInit();
        ccPressureInit();
    }

    if (Config::Sensors::MAX31856::ENABLED)
        thermocoupleInit();

    if (Config::Sensors::InternalADC::ENABLED)
        internalAdcInit();

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

InternalADCData Sensors::getInternalADCLastSample()
{
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}

ADS131M08Data Sensors::getADS131M08LastSample()
{
    return ads131m08 ? ads131m08->getLastSample() : ADS131M08Data{};
}

LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    return lis2mdl ? lis2mdl->getLastSample() : LIS2MDLData{};
}

LSM6DSRXData Sensors::getLSM6DSRXLastSample()
{
    return lsm6dsrx ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
}

PressureData Sensors::getRegulatorOutPressure()
{
    return regulatorOutPressure ? regulatorOutPressure->getLastSample()
                                : PressureData{};
}

PressureData Sensors::getOxTankTopPressure()
{
    return oxTankTopPressure ? oxTankTopPressure->getLastSample()
                             : PressureData{};
}

PressureData Sensors::getOxTankBottom0Pressure()
{
    return oxTankBottom0Pressure ? oxTankBottom0Pressure->getLastSample()
                                 : PressureData{};
}

PressureData Sensors::getOxTankBottom1Pressure()
{
    return oxTankBottom1Pressure ? oxTankBottom1Pressure->getLastSample()
                                 : PressureData{};
}

PressureData Sensors::getN2TankPressure()
{
    return n2TankPressure ? n2TankPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getCCPressure()
{
    return ccPressure ? ccPressure->getLastSample() : PressureData{};
}

TemperatureData Sensors::getThermocoupleTemperature()
{
    return thermocouple ? thermocouple->getLastSample() : TemperatureData{};
}

VoltageData Sensors::getBatteryVoltage()
{
    using namespace Config::Sensors::InternalADC;

    auto sample   = getInternalADCLastSample();
    float voltage = sample.voltage[(int)VBAT_CH] * VBAT_SCALE;
    return {sample.timestamp, voltage};
}

CurrentData Sensors::getActuatorsCurrent()
{
    using namespace Config::Sensors::InternalADC;

    auto sample   = getInternalADCLastSample();
    float current = sample.voltage[(int)ACTUATORS_CURRENT_CH] * CURRENT_SCALE +
                    CURRENT_OFFSET;

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

        PUSH_SENSOR_INFO(lps22df, "LPS22DF");
        PUSH_SENSOR_INFO(h3lis331dl, "H3LIS331DL");
        PUSH_SENSOR_INFO(lis2mdl, "LIS2MDL");
        PUSH_SENSOR_INFO(lsm6dsrx, "LSM6DSRX");
        PUSH_SENSOR_INFO(ads131m08, "ADS131M08");
        PUSH_SENSOR_INFO(internalAdc, "InternalADC");
        PUSH_SENSOR_INFO(regulatorOutPressure, "RegulatorOutPressure");
        PUSH_SENSOR_INFO(oxTankTopPressure, "OxTankTopPressure");
        PUSH_SENSOR_INFO(oxTankBottom0Pressure, "OxTankBottom0Pressure");
        PUSH_SENSOR_INFO(oxTankBottom1Pressure, "OxTankBottom1Pressure");
        PUSH_SENSOR_INFO(n2TankPressure, "N2TankPressure");
        PUSH_SENSOR_INFO(ccPressure, "CCPressure");
        PUSH_SENSOR_INFO(thermocouple, "Thermocouple");

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

void Sensors::lps22dfInit()
{
    SPIBusConfig spiConfig = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LPS22DF::Config config;
    config.avg = Config::Sensors::LPS22DF::AVG;
    config.odr = Config::Sensors::LPS22DF::ODR;

    lps22df = std::make_unique<LPS22DF>(getModule<Buses>()->getLPS22DF(),
                                        sensors::LPS22DF::cs::getPin(),
                                        spiConfig, config);
}

void Sensors::lps22dfCallback() { sdLogger.log(getLPS22DFLastSample()); }

void Sensors::h3lis331dlInit()
{
    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        getModule<Buses>()->getH3LIS331DL(), sensors::H3LIS331DL::cs::getPin(),
        spiConfig, Config::Sensors::H3LIS331DL::ODR,
        H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
        Config::Sensors::H3LIS331DL::FS);
}

void Sensors::h3lis331dlCallback() { sdLogger.log(getH3LIS331DLLastSample()); }

void Sensors::lis2mdlInit()
{
    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl = std::make_unique<LIS2MDL>(getModule<Buses>()->getLIS2MDL(),
                                        sensors::LIS2MDL::cs::getPin(),
                                        spiConfig, config);
}

void Sensors::lis2mdlCallback() { sdLogger.log(getLIS2MDLLastSample()); }

void Sensors::lsm6dsrxInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_0;

    LSM6DSRXConfig config;
    config.bdu = LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

    config.fsAcc     = Config::Sensors::LSM6DSRX::ACC_FS;
    config.odrAcc    = Config::Sensors::LSM6DSRX::ACC_ODR;
    config.opModeAcc = Config::Sensors::LSM6DSRX::ACC_OP_MODE;

    config.fsGyr     = Config::Sensors::LSM6DSRX::GYR_FS;
    config.odrGyr    = Config::Sensors::LSM6DSRX::GYR_ODR;
    config.opModeGyr = Config::Sensors::LSM6DSRX::GYR_OP_MODE;

    config.fifoMode = LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
    config.fifoTimestampDecimation =
        LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::HZ_52;

    lsm6dsrx = std::make_unique<LSM6DSRX>(getModule<Buses>()->getLSM6DSRX(),
                                          sensors::LSM6DSRX0::cs::getPin(),
                                          spiConfig, config);
}

void Sensors::lsm6dsrxCallback()
{
    if (!lsm6dsrx)
        return;

    uint16_t lastFifoSize;
    const auto lastFifo = lsm6dsrx->getLastFifo(lastFifoSize);

    // For every instance inside the fifo log the sample
    for (uint16_t i = 0; i < lastFifoSize; i++)
        sdLogger.log(lastFifo.at(i));
}

void Sensors::ads131m08Init()
{
    SPIBusConfig spiConfig;
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
    config.channelsConfig[(int)Config::Sensors::ADS131M08::N2_TANK_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::REGULATOR_OUT_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::OX_TANK_TOP_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::OX_TANK_BOTTOM_0_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::OX_TANK_BOTTOM_1_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::CC_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    ads131m08 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08(),
                                            sensors::ADS131M08::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::ads131m08Callback() { sdLogger.log(getADS131M08LastSample()); }

void Sensors::internalAdcInit()
{
    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(Config::Sensors::InternalADC::VBAT_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    sdLogger.log(getInternalADCLastSample());
}

void Sensors::regulatorOutPressureInit()
{
    regulatorOutPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::REGULATOR_OUT_PT_CHANNEL);
        },
        Config::Sensors::Trafag::REGULATOR_OUT_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::REGULATOR_OUT_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::regulatorOutPressureCallback()
{
    sdLogger.log(RegulatorOutPressureData{getRegulatorOutPressure()});
}

void Sensors::oxTankTopPressureInit()
{
    oxTankTopPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::OX_TANK_TOP_PT_CHANNEL);
        },
        Config::Sensors::Trafag::OX_TANK_TOP_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_TANK_TOP_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::oxTankTopPressureCallback()
{
    static uint32_t confidence = 0;

    auto sample = getOxTankTopPressure();

    if (sample.pressure > 64.f)
        confidence++;
    else
        confidence = 0;

    if (confidence > 100)
    {
        auto actuators = getModule<Actuators>();

        bool alreadyOpen = actuators->isServoOpen(ServosList::OX_VENTING_VALVE);
        if (!alreadyOpen)
            actuators->openServoWithTime(ServosList::OX_VENTING_VALVE, 1000);

        confidence = 0;
    }

    sdLogger.log(OxTankTopPressureData{sample});
}

void Sensors::oxTankBottomPressureInit()
{
    oxTankBottom0Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::OX_TANK_BOTTOM_0_PT_CHANNEL);
        },
        Config::Sensors::Trafag::OX_TANK_BOTTOM_0_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_TANK_BOTTOM_0_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);

    oxTankBottom1Pressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::OX_TANK_BOTTOM_1_PT_CHANNEL);
        },
        Config::Sensors::Trafag::OX_TANK_BOTTOM_1_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::OX_TANK_BOTTOM_1_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::oxTankBottom0PressureCallback()
{
    sdLogger.log(OxTankBottom0PressureData{getOxTankBottom0Pressure()});
}

void Sensors::oxTankBottom1PressureCallback()
{
    sdLogger.log(OxTankBottom1PressureData{getOxTankBottom1Pressure()});
}

void Sensors::n2TankPressureInit()
{
    n2TankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::N2_TANK_PT_CHANNEL);
        },
        Config::Sensors::Trafag::N2_TANK_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::N2_TANK_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::n2TankPressureCallback()
{
    sdLogger.log(N2TankPressureData{getN2TankPressure()});
}

void Sensors::ccPressureInit()
{
    ccPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(Config::Sensors::ADS131M08::CC_PT_CHANNEL);
        },
        Config::Sensors::Trafag::CC_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::CC_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::ccPressureCallback()
{
    sdLogger.log(CCPressureData{getCCPressure()});
}

void Sensors::thermocoupleInit()
{
    SPIBusConfig spiConfig = MAX31856::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    thermocouple = std::make_unique<MAX31856>(
        getModule<Buses>()->getThermocouple(),
        sensors::thermocouple::cs::getPin(), spiConfig,
        MAX31856::ThermocoupleType::K_TYPE);
}

void Sensors::thermocoupleCallback()
{
    if (!thermocouple)
        return;

    sdLogger.log(getThermocoupleTemperature());
}

bool Sensors::sensorManagerInit()
{
    SensorManager::SensorMap_t map;

    if (lps22df)
    {
        SensorInfo lps22dfInfo{"LPS22DF", Config::Sensors::LPS22DF::RATE,
                               [this]() { lps22dfCallback(); }};
        map.emplace(lps22df.get(), lps22dfInfo);
    }

    if (h3lis331dl)
    {
        SensorInfo h3lis331dlInfo{"H3LIS331DL",
                                  Config::Sensors::H3LIS331DL::RATE,
                                  [this]() { h3lis331dlCallback(); }};
        map.emplace(h3lis331dl.get(), h3lis331dlInfo);
    }

    if (lis2mdl)
    {
        SensorInfo lis2mdlInfo{"LIS2MDL", Config::Sensors::LIS2MDL::RATE,
                               [this]() { lis2mdlCallback(); }};
        map.emplace(lis2mdl.get(), lis2mdlInfo);
    }

    if (lsm6dsrx)
    {
        SensorInfo lsm6dsrxInfo{"LSM6DSRX", Config::Sensors::LSM6DSRX::RATE,
                                [this]() { lsm6dsrxCallback(); }};
        map.emplace(lsm6dsrx.get(), lsm6dsrxInfo);
    }

    if (ads131m08)
    {
        SensorInfo ads131m08Info{"ADS131M08", Config::Sensors::ADS131M08::RATE,
                                 [this]() { ads131m08Callback(); }};
        map.emplace(ads131m08.get(), ads131m08Info);
    }

    if (internalAdc)
    {
        SensorInfo internalAdcInfo{"InternalADC",
                                   Config::Sensors::InternalADC::RATE,
                                   [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), internalAdcInfo);
    }

    if (regulatorOutPressure)
    {
        SensorInfo info{"RegulatorOutPressure",
                        Config::Sensors::ADS131M08::RATE,
                        [this]() { regulatorOutPressureCallback(); }};
        map.emplace(std::make_pair(regulatorOutPressure.get(), info));
    }

    if (oxTankTopPressure)
    {
        SensorInfo info{"OxTopTankPressure", Config::Sensors::ADS131M08::RATE,
                        [this]() { oxTankTopPressureCallback(); }};
        map.emplace(std::make_pair(oxTankTopPressure.get(), info));
    }

    if (oxTankBottom0Pressure)
    {
        SensorInfo info{"OxBottomTank0Pressure",
                        Config::Sensors::ADS131M08::RATE,
                        [this]() { oxTankBottom0PressureCallback(); }};
        map.emplace(std::make_pair(oxTankBottom0Pressure.get(), info));
    }

    if (oxTankBottom1Pressure)
    {
        SensorInfo info{"OxBottomTank1Pressure",
                        Config::Sensors::ADS131M08::RATE,
                        [this]() { oxTankBottom1PressureCallback(); }};
        map.emplace(std::make_pair(oxTankBottom1Pressure.get(), info));
    }

    if (n2TankPressure)
    {
        SensorInfo info{"N2TankPressure", Config::Sensors::ADS131M08::RATE,
                        [this]() { n2TankPressureCallback(); }};
        map.emplace(std::make_pair(n2TankPressure.get(), info));
    }

    if (ccPressure)
    {
        SensorInfo info{"CCPressure", Config::Sensors::ADS131M08::RATE,
                        [this]() { ccPressureCallback(); }};
        map.emplace(std::make_pair(ccPressure.get(), info));
    }

    if (thermocouple)
    {
        SensorInfo info{"Thermocouple", Config::Sensors::MAX31856::PERIOD,
                        [this]() { thermocoupleCallback(); }};
        map.emplace(std::make_pair(thermocouple.get(), info));
    }

    manager = std::make_unique<SensorManager>(map, &getSensorsScheduler());
    return manager->start();
}
