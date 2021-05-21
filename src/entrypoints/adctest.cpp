/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/spi/SPIBus.h>

#include "Main/GlobalBuffers.h"
#include "configs/SensorManagerConfig.h"
#include "drivers/adc/ADS1118/ADS1118.h"
#include "TimestampTimer.h"

using namespace DeathStackBoard;
using namespace DeathStackBoard::SensorConfigs;
using namespace miosix;

ADS1118* adc_ads1118;
SPIBusInterface* spi1_bus = new SPIBus(SPI1);

void configure()
{
    SPIBusConfig spi_cfg = ADS1118::getDefaultSPIConfig();
    spi_cfg.clock_div    = SPIClockDivider::DIV128;

    ADS1118::ADS1118Config ads1118Config = ADS1118::ADS1118_DEFAULT_CONFIG;
    ads1118Config.bits.mode = ADS1118::ADS1118Mode::CONTIN_CONV_MODE;

    adc_ads1118 = new ADS1118(*spi1_bus, miosix::sensors::ads1118::cs::getPin(),
                              ads1118Config, spi_cfg);

    adc_ads1118->enableInput(ADC_CH_STATIC_PORT, ADC_DR_STATIC_PORT,
                             ADC_PGA_STATIC_PORT);

    adc_ads1118->enableInput(ADC_CH_PITOT_PORT, ADC_DR_PITOT_PORT,
                             ADC_PGA_PITOT_PORT);

    GlobalBuffers::buf_adc_pitot.reserve(GlobalBuffers::GLOBAL_BUF_LEN);
}

int main()
{
    miosix::xbee::reset::low();
    TimestampTimer::enableTimestampTimer();

    configure();

    int sample_period = 1000 / ADC_FS;
    while (GlobalBuffers::buf_adc_pitot.size() < GlobalBuffers::GLOBAL_BUF_LEN)
    {
        long long start = miosix::getTick();

        GlobalBuffers::buf_adc_pitot.push_back(adc_ads1118->getLastSample());
        adc_ads1118->sample();

        Thread::sleepUntil(start + sample_period);
    }

    printf("timestamp,voltage_pitot\n");
    for (auto v : GlobalBuffers::buf_adc_pitot)
    {
        printf("%llu,%f\n", v.adc_timestamp, v.voltage);
    }

    for (;;)
    {
        Thread::sleep(1000);
    }
}