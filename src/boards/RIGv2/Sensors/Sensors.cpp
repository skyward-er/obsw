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

#include <RIGv2/Buses.h>
#include <RIGv2/Configs/SensorsConfig.h>
#include <RIGv2/Sensors/SensorsData.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace miosix;
using namespace RIGv2;

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    SensorManager::SensorMap_t map;
    adc1Init(map);

    manager = std::make_unique<SensorManager>(map, &scheduler);
    if (!manager->start())
    {
        LOG_ERR(logger, "Failed to start SensorManager");
        return false;
    }

    started = true;
    return true;
}

void Sensors::stop()
{
    manager->stop();
    started = false;
}

ADS131M08Data Sensors::getADC1LastSample()
{
    PauseKernelLock l;
    return adc1->getLastSample();
}

void Sensors::adc1Init(SensorManager::SensorMap_t &map)
{
    ModuleManager &modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;

    ADS131M08::Config config     = {};
    config.oversamplingRatio     = ADS131M08Defs::OversamplingRatio::OSR_8192;
    config.globalChopModeEnabled = true;

    adc1 = std::make_unique<ADS131M08>(modules.get<Buses>()->getADS131M08_1(),
                                       sensors::ADS131_1::cs::getPin(),
                                       spiConfig, config);

    SensorInfo info("ADS131M08_1", Config::Sensors::ADC_SAMPLE_PERIOD,
                    [this]() { adc1Callback(); });
    map.emplace(std::make_pair(adc1.get(), info));
}

void Sensors::adc1Callback()
{
    ADS131M08Data sample = adc1->getLastSample();

    ADCsData data{sample.timestamp,  1,
                  sample.voltage[0], sample.voltage[1],
                  sample.voltage[2], sample.voltage[3],
                  sample.voltage[4], sample.voltage[5],
                  sample.voltage[6], sample.voltage[7]};

    // For Flavio, fuck Flavio
    LOG_INFO(logger, "{}\t{}", sample.voltage[6], sample.voltage[7]);

    sdLogger.log(data);
}