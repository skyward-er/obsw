/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Motor/Buses.h>
#include <Motor/Configs/SensorsConfig.h>
#include <Motor/Sensors/SensorsData.h>
#include <interfaces-impl/hwmapping.h>

using namespace Motor;
using namespace Boardcore;
using namespace miosix;

bool Sensors::start()
{
    if (Config::Sensors::LPS22DF::ENABLED)
    {
        lps22dfInit();
    }

    if (Config::Sensors::H3LIS331DL::ENABLED)
    {
        h3lis331dlInit();
    }

    if (Config::Sensors::LIS2MDL::ENABLED)
    {
        lis2mdlInit();
    }

    if (Config::Sensors::LSM6DSRX::ENABLED)
    {
        lsm6dsrxInit();
    }

    if (Config::Sensors::ADS131M08::ENABLED)
    {
        ads131m08Init();
        topTankPressureInit();
        ccPressureInit();
    }

    if (Config::Sensors::InternalADC::ENABLED)
    {
        internalAdcInit();
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

PressureData Sensors::getTopTankPress()
{
    return topTankPressure ? topTankPressure->getLastSample() : PressureData{};
}

PressureData Sensors::getCCPress()
{
    return ccPressure ? ccPressure->getLastSample() : PressureData{};
}

TemperatureData Sensors::getTankTemp() { return {}; }

VoltageData Sensors::getBatteryVoltage()
{
    auto sample   = getInternalADCLastSample();
    float voltage = sample.voltage[(int)Config::Sensors::InternalADC::VBAT_CH] *
                    Config::Sensors::InternalADC::VBAT_SCALE;
    return {sample.timestamp, voltage};
}

std::vector<SensorInfo> Sensors::getSensorInfo()
{
    if (manager)
    {
        return {manager->getSensorInfo(lps22df.get()),
                manager->getSensorInfo(h3lis331dl.get()),
                manager->getSensorInfo(lis2mdl.get()),
                manager->getSensorInfo(lsm6dsrx.get()),
                manager->getSensorInfo(ads131m08.get()),
                manager->getSensorInfo(internalAdc.get()),
                manager->getSensorInfo(topTankPressure.get()),
                manager->getSensorInfo(ccPressure.get())};
    }
    else
    {
        return {};
    }
}

void Sensors::lps22dfInit()
{
    if (!Config::Sensors::LPS22DF::ENABLED)
        return;

    SPIBusConfig spiConfig = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LPS22DF::Config config;
    config.avg = Config::Sensors::LPS22DF::AVG;
    config.odr = Config::Sensors::LPS22DF::ODR;

    lps22df = std::make_unique<LPS22DF>(getModule<Buses>()->getLPS22DF(),
                                        sensors::LPS22DF::cs::getPin(),
                                        spiConfig, config);
}

void Sensors::lps22dfCallback() { sdLogger.log(lps22df->getLastSample()); }

void Sensors::h3lis331dlInit()
{
    if (!Config::Sensors::H3LIS331DL::ENABLED)
        return;

    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        getModule<Buses>()->getH3LIS331DL(), sensors::H3LIS331DL::cs::getPin(),
        spiConfig, Config::Sensors::H3LIS331DL::ODR,
        H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
        Config::Sensors::H3LIS331DL::FS);
}

void Sensors::h3lis331dlCallback()
{
    sdLogger.log(h3lis331dl->getLastSample());
}

void Sensors::lis2mdlInit()
{
    if (!Config::Sensors::LIS2MDL::ENABLED)
        return;

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

void Sensors::lis2mdlCallback() { sdLogger.log(lis2mdl->getLastSample()); }

void Sensors::lsm6dsrxInit()
{
    if (!Config::Sensors::LSM6DSRX::ENABLED)
        return;

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
    config.fifoTemperatureBdr = LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

    lsm6dsrx = std::make_unique<LSM6DSRX>(getModule<Buses>()->getLSM6DSRX(),
                                          sensors::LSM6DSRX::cs::getPin(),
                                          spiConfig, config);
}

void Sensors::lsm6dsrxCallback() { sdLogger.log(lsm6dsrx->getLastSample()); }

void Sensors::ads131m08Init()
{
    if (!Config::Sensors::ADS131M08::ENABLED)
        return;

    SPIBusConfig spiConfig;
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
    config.channelsConfig[(
        int)Config::Sensors::ADS131M08::TANK_TOP_PT_CHANNEL] = {
        .enabled = true,
        .pga     = ADS131M08Defs::PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0};

    config.channelsConfig[(int)Config::Sensors::ADS131M08::ENGINE_PT_CHANNEL] =
        {.enabled = true,
         .pga     = ADS131M08Defs::PGA::PGA_1,
         .offset  = 0,
         .gain    = 1.0};

    ads131m08 = std::make_unique<ADS131M08>(getModule<Buses>()->getADS131M08(),
                                            sensors::ADS131M08::cs::getPin(),
                                            spiConfig, config);
}

void Sensors::ads131m08Callback() { sdLogger.log(ads131m08->getLastSample()); }

void Sensors::internalAdcInit()
{
    if (!Config::Sensors::InternalADC::ENABLED)
        return;

    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(InternalADC::CH9);
    internalAdc->enableChannel(InternalADC::CH14);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();
}

void Sensors::internalAdcCallback()
{
    sdLogger.log(internalAdc->getLastSample());
}

void Sensors::topTankPressureInit()
{
    if (!Config::Sensors::ADS131M08::ENABLED)
        return;

    topTankPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::TANK_TOP_PT_CHANNEL);
        },
        Config::Sensors::Trafag::TANK_TOP_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::TANK_TOP_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::topTankPressureCallback()
{
    PressureData sample = topTankPressure->getLastSample();
    PTsData data{sample.pressureTimestamp, 0, sample.pressure};
    sdLogger.log(data);
}

void Sensors::ccPressureInit()
{
    if (!Config::Sensors::ADS131M08::ENABLED)
        return;

    ccPressure = std::make_unique<TrafagPressureSensor>(
        [this]()
        {
            auto sample = getADS131M08LastSample();
            return sample.getVoltage(
                Config::Sensors::ADS131M08::ENGINE_PT_CHANNEL);
        },
        Config::Sensors::Trafag::ENGINE_SHUNT_RESISTANCE,
        Config::Sensors::Trafag::ENGINE_MAX_PRESSURE,
        Config::Sensors::Trafag::MIN_CURRENT,
        Config::Sensors::Trafag::MAX_CURRENT);
}

void Sensors::ccPressureCallback()
{
    PressureData sample = topTankPressure->getLastSample();
    PTsData data{sample.pressureTimestamp, 1, sample.pressure};
    sdLogger.log(data);
}

bool Sensors::sensorManagerInit()
{
    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getSensorsScheduler();

    SensorManager::SensorMap_t map;

    if (lps22df)
    {
        SensorInfo lps22dfInfo{"LPS22DF", Config::Sensors::LPS22DF::PERIOD,
                               [this]() { lps22dfCallback(); }};
        map.emplace(lps22df.get(), lps22dfInfo);
    }

    if (h3lis331dl)
    {
        SensorInfo h3lis331dlInfo{"H3LIS331DL",
                                  Config::Sensors::H3LIS331DL::PERIOD,
                                  [this]() { h3lis331dlCallback(); }};
        map.emplace(h3lis331dl.get(), h3lis331dlInfo);
    }

    if (lis2mdl)
    {
        SensorInfo lis2mdlInfo{"LIS2MDL", Config::Sensors::LIS2MDL::PERIOD,
                               [this]() { lis2mdlCallback(); }};
        map.emplace(lis2mdl.get(), lis2mdlInfo);
    }

    if (lsm6dsrx)
    {
        SensorInfo lsm6dsrxInfo{"LSM6DSRX", Config::Sensors::LSM6DSRX::PERIOD,
                                [this]() { lsm6dsrxCallback(); }};
        map.emplace(lsm6dsrx.get(), lsm6dsrxInfo);
    }

    if (ads131m08)
    {
        SensorInfo ads131m08Info{"ADS131M08",
                                 Config::Sensors::ADS131M08::PERIOD,
                                 [this]() { ads131m08Callback(); }};
        map.emplace(ads131m08.get(), ads131m08Info);
    }

    if (internalAdc)
    {
        SensorInfo internalAdcInfo{"InternalADC",
                                   Config::Sensors::InternalADC::PERIOD,
                                   [this]() { internalAdcCallback(); }};
        map.emplace(internalAdc.get(), internalAdcInfo);
    }

    if (topTankPressure)
    {
        SensorInfo info("TopTankPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { topTankPressureCallback(); });
        map.emplace(std::make_pair(topTankPressure.get(), info));
    }

    if (ccPressure)
    {
        SensorInfo info("CCPressure", Config::Sensors::ADS131M08::PERIOD,
                        [this]() { ccPressureCallback(); });
        map.emplace(std::make_pair(ccPressure.get(), info));
    }

    manager = std::make_unique<SensorManager>(map, &scheduler);
    return manager->start();
}