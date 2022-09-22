/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Ciuti/Buses.h>
#include <Ciuti/Configs/SensorsConfig.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Ciuti::SensorsConfig;

namespace Ciuti
{

bool Sensors::start() { return sensorManager->start(); }

Boardcore::InternalADCData Sensors::getInternalADCLastSample(
    InternalADC::Channel channel)
{
    PauseKernelLock lock;
    return internalAdc->getVoltage(channel);
}

Boardcore::LIS331HHData Sensors::getLIS331HHLastSample()
{
    PauseKernelLock lock;
    return lis331hh->getLastSample();
}

Sensors::Sensors()
{
    internalAdcInit();
    lis331hhInit();

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);
}

Sensors::~Sensors()
{
    delete internalAdc;
    delete lis331hh;
}

void Sensors::internalAdcInit()
{
    internalAdc = new InternalADC(ADC3, INTERNAL_ADC_VREF);

    internalAdc->enableChannel(INTERNAL_ADC_CH_0);
    internalAdc->enableChannel(INTERNAL_ADC_CH_1);

    devices::ina188::mosfet1::high();
    devices::ina188::mosfet2::high();

    SensorInfo info("INTERNAL_ADC", SAMPLE_PERIOD_INTERNAL_ADC,
                    [=]()
                    {
                        Logger::getInstance().log(
                            internalAdc->getVoltage(INTERNAL_ADC_CH_0));
                        Logger::getInstance().log(
                            internalAdc->getVoltage(INTERNAL_ADC_CH_1));
                    });

    sensorsMap.emplace(std::make_pair(internalAdc, info));

    LOG_INFO(logger, "Internal ADC setup done!");
}

void Sensors::lis331hhInit()
{
    SPIBusConfig config;

    lis331hh = new LIS331HH(Buses::getInstance().spi2,
                            devices::lis331hh::cs::getPin(), config);

    lis331hh->init();
    lis331hh->setOutputDataRate(LIS331HH::ODR_50);
    lis331hh->setFullScaleRange(LIS331HH::FS_24);

    SensorInfo info("LIS331HH", SAMPLE_PERIOD_LIS331HH,
                    [=]()
                    { Logger::getInstance().log(lis331hh->getLastSample()); });

    sensorsMap.emplace(std::make_pair(lis331hh, info));

    LOG_INFO(logger, "LIS331HH setup done!");
}

}  // namespace Ciuti
