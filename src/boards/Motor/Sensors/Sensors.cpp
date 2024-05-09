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
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Motor;
using namespace Boardcore;
using namespace miosix;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    SensorManager::SensorMap_t map;
    lps22dfInit(map);
    h3lis331dlInit(map);
    lis2mdlInit(map);
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

std::vector<SensorInfo> Sensors::getSensorInfo()
{
    if (manager)
    {
        return {manager->getSensorInfo(lps22df.get()),
                manager->getSensorInfo(h3lis331dl.get()),
                manager->getSensorInfo(lis2mdl.get()),
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

    ads131m08 = std::make_unique<ADS131M08>(
        modules.get<Buses>()->getADS131M08(), sensors::ADS131M08::cs::getPin(),
        spiConfig, config);

    SensorInfo info{"ADS131M08", 1000, [this]() { ads131m08Callback(); }};
    map.emplace(ads131m08.get(), info);
}

float voltageToTemp(float voltage)
{
    // Two point calibration
    // m = dmass/dvoltage
    float scale  = (175 + 40) / (0.996 - 0.1);
    float offset = -40 - scale * 0.1;  // Calculate offset
    return scale * voltage + offset;
}

float voltageToPress(float voltage)
{
    // First convert voltage to current
    float current = (voltage / Config::Sensors::ADS131M08::CH5_SHUNT_RESISTANCE) * 1000.0f;

    // Convert to a value between 0 and 1
    float value = (current - 4) / (20 - 4);

    // Scale from 0 to maxPressure
    return value * 40;
}

void Sensors::ads131m08Callback()
{
    auto sample = ads131m08->getLastSample();

    LOG_INFO(logger, "ADC: {}\t{}", voltageToPress(sample.voltage[5]), voltageToPress(sample.voltage[6]));
}

void Sensors::internalAdcInit(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    internalAdc = std::make_unique<InternalADC>(ADC2);
    internalAdc->enableChannel(InternalADC::CH9);
    internalAdc->enableChannel(InternalADC::CH14);
    internalAdc->enableTemperature();
    internalAdc->enableVbat();

    SensorInfo info{"InternalADC", Config::Sensors::InternalADC::PERIOD,
                    [this]() { internalAdcCallback(); }};
    map.emplace(internalAdc.get(), info);
}

void Sensors::internalAdcCallback() {}