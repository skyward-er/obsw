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

    if (Config::Sensors::ADC_1::ENABLED)
        adc1Init();

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

ADS131M08Data Sensors::getADC1LastSample()
{
    return adc1 ? adc1->getLastSample() : ADS131M08Data{};
}

ADS131M08Data Sensors::getADC2LastSample()
{
    return adc2 ? adc2->getLastSample() : ADS131M08Data{};
}

ADS131M08Data Sensors::getADC3LastSample()
{
    return adc3 ? adc3->getLastSample() : ADS131M08Data{};
}

// need to figure out how to handle this with 2 sensor managers
std::vector<SensorInfo> Sensors::getSensorInfos()
{
    if (SPI2Manager)
    {
        std::vector<SensorInfo> infos{};

#define PUSH_SENSOR_INFO(manager, instance, name)                \
    if (instance)                                                \
        infos.push_back(manager->getSensorInfo(instance.get())); \
    else                                                         \
        infos.push_back(SensorInfo{name, 0ns, nullptr, false})

        PUSH_SENSOR_INFO(SPI2Manager, adc0, "ADS131M08_0");
        PUSH_SENSOR_INFO(SPI2Manager, adc1, "ADS131M08_1");
        PUSH_SENSOR_INFO(SPI3Manager, adc2, "ADS131M08_2");
        PUSH_SENSOR_INFO(SPI3Manager, adc3, "ADS131M08_3");
        PUSH_SENSOR_INFO(SPI3Manager, internalAdc, "InternalADC");
#undef PUSH_SENSOR_INFO
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
    config.oversamplingRatio = Config::Sensors::ADS131M08_FAST::OSR;
    config.globalChopModeEnabled =
        Config::Sensors::ADS131M08_FAST::GLOBAL_CHOP_MODE_EN;

    // Disable all channels
    for (auto& channel : config.channelsConfig)
    {
        channel = {.enabled = true,
                   .pga     = ADS131M08Defs::PGA::PGA_1,
                   .offset  = 0,
                   .gain    = 1.0};
    }

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC0(),
                                       getModule<Buses>()->getADC0CsPin(),
                                       spiConfig, config);
}

void Sensors::adc1Init()
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
    {
        channel = {.enabled = true,
                   .pga     = ADS131M08Defs::PGA::PGA_1,
                   .offset  = 0,
                   .gain    = 1.0};
    }

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC1(),
                                       getModule<Buses>()->getADC1CsPin(),
                                       spiConfig, config);
}
void Sensors::adc2Init()
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
    {
        channel = {.enabled = true,
                   .pga     = ADS131M08Defs::PGA::PGA_1,
                   .offset  = 0,
                   .gain    = 1.0};
    }

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC2(),
                                       getModule<Buses>()->getADC2CsPin(),
                                       spiConfig, config);
}

void Sensors::adc3Init()
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
    {
        channel = {.enabled = true,
                   .pga     = ADS131M08Defs::PGA::PGA_1,
                   .offset  = 0,
                   .gain    = 1.0};
    }

    adc1 = std::make_unique<ADS131M08>(getModule<Buses>()->getADC3(),
                                       getModule<Buses>()->getADC3CsPin(),
                                       spiConfig, config);
}

void Sensors::adc0Callback() { sdLogger.log(ADC0Data{getADC0LastSample()}); }

void Sensors::adc1Callback() { sdLogger.log(ADC1Data{getADC1LastSample()}); }

void Sensors::adc2Callback() { sdLogger.log(ADC2Data{getADC2LastSample()}); }

void Sensors::adc3Callback() { sdLogger.log(ADC3Data{getADC3LastSample()}); }

bool Sensors::sensorManagerInit()
{
    TaskScheduler& SPI2scheduler = getModule<BoardScheduler>()->SPI2sensors();
    TaskScheduler& SPI3scheduler = getModule<BoardScheduler>()->SPI3sensors();

    SensorManager::SensorMap_t SPI2map;
    SensorManager::SensorMap_t SPI3map;

    if (adc0)
    {
        SensorInfo info("ADC0", Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { adc0Callback(); });
        SPI2map.emplace(std::make_pair(adc0.get(), info));
    }

    if (adc1)
    {
        SensorInfo info("ADC1", Config::Sensors::ADS131M08_FAST::PERIOD,
                        [this]() { adc1Callback(); });
        SPI2map.emplace(std::make_pair(adc1.get(), info));
    }

    if (adc2)
    {
        SensorInfo info("ADC2", Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { adc2Callback(); });
        SPI3map.emplace(std::make_pair(adc2.get(), info));
    }

    if (adc3)
    {
        SensorInfo info("ADC3", Config::Sensors::ADS131M08_SLOW::PERIOD,
                        [this]() { adc3Callback(); });
        SPI3map.emplace(std::make_pair(adc3.get(), info));
    }

    if (internalAdc)
    {
        SensorInfo info("InternalADC", Config::Sensors::InternalADC::PERIOD,
                        [this]() { internalAdcCallback(); });
        SPI3map.emplace(internalAdc.get(), info);
    }

    SPI2Manager = std::make_unique<SensorManager>(SPI2map, &SPI2scheduler);
    SPI3Manager = std::make_unique<SensorManager>(SPI3map, &SPI3scheduler);
    return SPI2Manager->start() && SPI3Manager->start();
}
