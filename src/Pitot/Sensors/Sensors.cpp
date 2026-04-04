/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Leonardo Montecchi
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
#include "SensorData.h"

#include <Pitot/BoardScheduler.h>
#include <Pitot/Buses.h>
#include <Pitot/Configs/SensorsConfig.h>
#include <common/ReferenceConfig.h>
#include <interfaces-impl/hwmapping.h>

#include <chrono>
#include <mutex>

using namespace std::chrono;
using namespace miosix;
using namespace Eigen;
using namespace Boardcore;

namespace Pitot
{

bool Sensors::isStarted() { return started; }

bool Sensors::start()
{
    if (Config::Sensors::HeatingPadNTC::ENABLED)
        internalADCInit();

    if (Config::Sensors::ND015A::ENABLED)
        nd015aInit();

    if (Config::Sensors::ND030D::ENABLED)
        nd030dInit();

    // Return immediately if the hook fails as we cannot know what the hook does
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

    started = true;
    return true;
}

void Sensors::calibrate()
{
    float dynPressureAccum = 0.0f;

    for (int i = 0; i < Config::Sensors::CALIBRATION_SAMPLES_COUNT; i++)
    {
        auto dynPressure = getDynamicPressureLastSample();

        dynPressureAccum += dynPressure.pressure;

        Thread::sleep(
            milliseconds{Config::Sensors::CALIBRATION_SLEEP_TIME}.count());
    }

    float dynPressureOffset =
        dynPressureAccum / Config::Sensors::CALIBRATION_SAMPLES_COUNT;

    nd030d->updateOffset(dynPressureOffset);
}

Boardcore::VoltageData Sensors::getHeatingPadNTCVoltageLastSample()
{
    auto sample   = getinternalADCLastSample();
    float voltage = sample.voltage[(int)Config::Sensors::HeatingPadNTC::CH];
    return {sample.timestamp, voltage};
}

Boardcore::TemperatureData Sensors::getHeatingPadNTCTemperatureLastSample()
{
    auto sample = getinternalADCLastSample();
    float voltage = sample.voltage[(int)Config::Sensors::HeatingPadNTC::CH];

    float resistance = (Config::Sensors::HeatingPadNTC::REF_RESISTANCE *
                        voltage) /
                       (Config::Sensors::HeatingPadNTC::REF_VOLTAGE - voltage);

    float temperature = 1.0f /
                        ((1.0f / Config::Sensors::HeatingPadNTC::REF_TEMPERATURE) +
                         (1.0f / Config::Sensors::HeatingPadNTC::BETA) *
                             std::log(resistance /
                                      Config::Sensors::HeatingPadNTC::
                                          REF_RESISTANCE));

    return {sample.timestamp, temperature};
}

ND015XData Sensors::getND015ADataLastSample()
{
    return nd015a ? nd015a->getLastSample() : ND015XData{};
}

ND030XData Sensors::getND030DDataLastSample()
{
    return nd030d ? nd030d->getLastSample() : ND030XData{};
}

StaticPressureData Sensors::getStaticPressureLastSample()
{
    return StaticPressureData{getND015ADataLastSample()};
}

DynamicPressureData Sensors::getDynamicPressureLastSample()
{
    return DynamicPressureData{getND030DDataLastSample()};
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
        infos.push_back(SensorInfo{name, 0, nullptr, false})
        PUSH_SENSOR_INFO(nd015a, "ND015A");
        PUSH_SENSOR_INFO(nd030d, "ND030D");

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

void Sensors::internalADCInit(){
    internalADC = std::make_unique<InternalADC>(ADC2);
    internalADC->enableChannel(Config::Sensors::HeatingPadNTC::CH);
}
void Sensors::internalADCCallback(){

}

void Sensors::nd015aInit()
{
    SPIBusConfig spiConfig = ND015A::getDefaultSPIConfig();

    nd015a = std::make_unique<ND015A>(
        getModule<Buses>()->getND015A(), sensors::ND015A::cs::getPin(),
        spiConfig, Config::Sensors::ND015A::IOW, Config::Sensors::ND015A::BWL,
        Config::Sensors::ND015A::NTC, Config::Sensors::ND015A::ODR);
}

void Sensors::nd015aCallback()
{
    sdLogger.log(StaticPressureData{getND015ADataLastSample()});
}

void Sensors::nd030dInit()
{
    SPIBusConfig spiConfig = ND030D::getDefaultSPIConfig();

    nd030d = std::make_unique<ND030D>(
        getModule<Buses>()->getND030D(), sensors::ND030D::cs::getPin(),
        spiConfig, Config::Sensors::ND030D::FSR, Config::Sensors::ND030D::IOW,
        Config::Sensors::ND030D::BWL, Config::Sensors::ND030D::NTC,
        Config::Sensors::ND030D::ODR);
}

void Sensors::nd030dCallback()
{
    sdLogger.log(DynamicPressureData{getND030DDataLastSample()});
}

bool Sensors::sensorManagerInit()
{
    SensorManager::SensorMap_t map;

    if (nd015a)
    {
        SensorInfo info{"ND015A", Config::Sensors::ND015A::RATE,
                        [this]() { nd015aCallback(); }};
        map.emplace(nd015a.get(), info);
    }

    if (nd030d)
    {
        SensorInfo info{"ND030D", Config::Sensors::ND030D::RATE,
                        [this]() { nd030dCallback(); }};
        map.emplace(nd030d.get(), info);
    }

    manager = std::make_unique<SensorManager>(map, &getSensorsScheduler());
    return manager->start();
}

}  // namespace Pitot
