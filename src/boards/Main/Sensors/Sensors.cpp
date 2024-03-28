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

#include <Main/Buses.h>
#include <Main/Configs/SensorsConfig.h>
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Main;
using namespace Boardcore;
using namespace miosix;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    SensorManager::SensorMap_t map;
    lps22dfInit(map);
    lps28dfwInit(map);
    h3lis331dlInit(map);
    lis2mdlInit(map);
    ubxgpsInit(map);
    lsm6dsrxInit(map);
    ads131m08Init(map);
    internalAdcInit(map);

    manager = std::make_unique<SensorManager>(map, &scheduler);
    if (!manager->start())
    {
        return false;
    }

    started = true;
    return true;
}

Boardcore::LPS22DFData Sensors::getLPS22DFLastSample()
{
    return lps22df ? lps22df->getLastSample() : LPS22DFData{};
}

Boardcore::LPS28DFWData Sensors::getLPS28DFWLastSample()
{
    return lps28dfw ? lps28dfw->getLastSample() : LPS28DFWData{};
}

Boardcore::H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    return h3lis331dl ? h3lis331dl->getLastSample() : H3LIS331DLData{};
}

Boardcore::LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    return lis2mdl ? lis2mdl->getLastSample() : LIS2MDLData{};
}

Boardcore::UBXGPSData Sensors::getUBXGPSLastSample()
{
    return ubxgps ? ubxgps->getLastSample() : UBXGPSData{};
}

Boardcore::LSM6DSRXData Sensors::getLSM6DSRXLastSample()
{
    return lsm6dsrx ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
}

Boardcore::ADS131M08Data Sensors::getADS131M08LastSample()
{
    return ads131m08 ? ads131m08->getLastSample() : ADS131M08Data{};
}

Boardcore::InternalADCData Sensors::getInternalADCLastSample()
{
    return internalAdc ? internalAdc->getLastSample() : InternalADCData{};
}

Boardcore::VoltageData Sensors::getBatteryVoltage()
{
    auto sample   = getInternalADCLastSample();
    float voltage = sample.voltage[(int)Config::Sensors::InternalADC::VBAT_CH] *
                    Config::Sensors::InternalADC::VBAT_SCALE;
    return {sample.timestamp, voltage};
}

Boardcore::VoltageData Sensors::getCamBatteryVoltage()
{
    auto sample = getInternalADCLastSample();
    float voltage =
        sample.voltage[(int)Config::Sensors::InternalADC::CAM_VBAT_CH] *
        Config::Sensors::InternalADC::CAM_VBAT_SCALE;
    return {sample.timestamp, voltage};
}

std::vector<Boardcore::SensorInfo> Sensors::getSensorInfo()
{
    if (manager)
    {
        return {manager->getSensorInfo(lps22df.get()),
                manager->getSensorInfo(lps28dfw.get()),
                manager->getSensorInfo(h3lis331dl.get()),
                manager->getSensorInfo(lis2mdl.get()),
                manager->getSensorInfo(ubxgps.get()),
                manager->getSensorInfo(lsm6dsrx.get()),
                manager->getSensorInfo(ads131m08.get()),
                manager->getSensorInfo(internalAdc.get())};
    }
    else
    {
        return {};
    }
}

void Sensors::lps22dfInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = LPS22DF::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LPS22DF::Config config;
    config.avg = Config::Sensors::LPS22DF::AVG;
    config.odr = Config::Sensors::LPS22DF::ODR;

    lps22df = std::make_unique<LPS22DF>(modules.get<Buses>()->getLPS22DF(),
                                        sensors::LPS22DF::cs::getPin(),
                                        spiConfig, config);

    SensorInfo info{"LPS22DF", Config::Sensors::LPS22DF::PERIOD,
                    [this]() { lps22dfCallback(); }};
    map.emplace(lps22df.get(), info);
}

void Sensors::lps22dfCallback() {}

void Sensors::lps28dfwInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    LPS28DFW::SensorConfig config;
    config.sa0  = true;
    config.fsr  = Config::Sensors::LPS28DFW::FS;
    config.avg  = Config::Sensors::LPS28DFW::AVG;
    config.odr  = Config::Sensors::LPS28DFW::ODR;
    config.drdy = false;

    lps28dfw =
        std::make_unique<LPS28DFW>(modules.get<Buses>()->getLPS28DFW(), config);

    SensorInfo info{"LPS28DFW", Config::Sensors::LPS28DFW::PERIOD,
                    [this]() { lps28dfwCallback(); }};
    map.emplace(lps28dfw.get(), info);
}

void Sensors::lps28dfwCallback() {}

void Sensors::h3lis331dlInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    h3lis331dl = std::make_unique<H3LIS331DL>(
        modules.get<Buses>()->getH3LIS331DL(),
        sensors::H3LIS331DL::cs::getPin(), spiConfig,
        Config::Sensors::H3LIS331DL::ODR,
        H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
        Config::Sensors::H3LIS331DL::FS);

    SensorInfo info{"H3LIS331DL", Config::Sensors::H3LIS331DL::PERIOD,
                    [this]() { h3lis331dlCallback(); }};
    map.emplace(h3lis331dl.get(), info);
}

void Sensors::h3lis331dlCallback() {}

void Sensors::lis2mdlInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = H3LIS331DL::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;

    LIS2MDL::Config config;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.odr                = Config::Sensors::LIS2MDL::ODR;
    config.temperatureDivider = Config::Sensors::LIS2MDL::TEMP_DIVIDER;

    lis2mdl = std::make_unique<LIS2MDL>(modules.get<Buses>()->getLIS2MDL(),
                                        sensors::LIS2MDL::cs::getPin(),
                                        spiConfig, config);

    SensorInfo info{"LIS2MDL", Config::Sensors::LIS2MDL::PERIOD,
                    [this]() { lis2mdlCallback(); }};
    map.emplace(lis2mdl.get(), info);
}

void Sensors::lis2mdlCallback() {}

void Sensors::ubxgpsInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = UBXGPSSpi::getDefaultSPIConfig();
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;

    ubxgps = std::make_unique<UBXGPSSpi>(modules.get<Buses>()->getUBXGps(),
                                         sensors::UBXGps::cs::getPin(),
                                         spiConfig, 5);

    SensorInfo info{"UBXGPS", Config::Sensors::UBXGPS::PERIOD,
                    [this]() { ubxgpsCallback(); }};
    map.emplace(ubxgps.get(), info);
}

void Sensors::ubxgpsCallback() {}

void Sensors::lsm6dsrxInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

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

    lsm6dsrx = std::make_unique<LSM6DSRX>(modules.get<Buses>()->getLSM6DSRX(),
                                          sensors::LSM6DSRX::cs::getPin(),
                                          spiConfig, config);

    SensorInfo info{"LSM6DSRX", Config::Sensors::LSM6DSRX::PERIOD,
                    [this]() { lsm6dsrxCallback(); }};
    map.emplace(lsm6dsrx.get(), info);
}

void Sensors::lsm6dsrxCallback() {}

void Sensors::ads131m08Init(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config;
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

    // Enable required channels
    config.channelsConfig[0].enabled = true;
    config.channelsConfig[1].enabled = true;
    config.channelsConfig[2].enabled = true;

    ads131m08 = std::make_unique<ADS131M08>(
        modules.get<Buses>()->getADS131M08(), sensors::ADS131M08::cs::getPin(),
        spiConfig, config);

    SensorInfo info{"ADS131M08", 2000, [this]() { ads131m08Callback(); }};
    map.emplace(ads131m08.get(), info);
}

void Sensors::ads131m08Callback()
{
    auto sample = ads131m08->getLastSample();
    LOG_ERR(logger, "\n{} {} {}", sample.voltage[0], sample.voltage[1],
            sample.voltage[2]);
}

void Sensors::internalAdcInit(Boardcore::SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(Config::Sensors::InternalADC::VBAT_CH);
    internalAdc->enableChannel(Config::Sensors::InternalADC::CAM_VBAT_CH);
    internalAdc->enableChannel(Config::Sensors::InternalADC::CUTTER_SENSE_CH);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();

    SensorInfo info{"InternalADC", Config::Sensors::InternalADC::PERIOD,
                    [this]() { internalAdcCallback(); }};
    map.emplace(internalAdc.get(), info);
}

void Sensors::internalAdcCallback() {}
