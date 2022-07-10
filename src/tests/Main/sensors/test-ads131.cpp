/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/ADS131M04/ADS131M04.h>
#include <utils/Stats/Stats.h>

using namespace miosix;
using namespace Boardcore;

constexpr float VOLTAGE_DIVIDER = 2.3 / (10 + 2.3);

int main()
{
    // SPI configuration setup
    SPIBus spiBus(SPI1);
    SPIBusConfig spiConfig = {};
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;
    SPISlave spiSlave(spiBus, sensors::ads131m04::cs1::getPin(), spiConfig);

    // Device initialization
    ADS131M04 ads131(spiSlave);

    // Initialize the device
    if (!ads131.init())
        printf("Init failed\n");

    ads131.enableChannel(ADS131M04::Channel::CHANNEL_0);
    ads131.enableChannel(ADS131M04::Channel::CHANNEL_1);
    ads131.enableChannel(ADS131M04::Channel::CHANNEL_2);
    ads131.enableChannel(ADS131M04::Channel::CHANNEL_3);
    ads131.enableGlobalChopMode();
    ads131.setOversamplingRatio(ADS131M04::OversamplingRatio::OSR_4096);

    miosix::delayMs(1000);

    // Calculating statistics of the sensor
    const int nSamples = 100;
    Stats stats[4];

    printf("Calculating statistics...\n");

    for (int iSample = 0; iSample < nSamples; iSample++)
    {
        ads131.sample();

        ADS131M04Data data = ads131.getLastSample();
        float voltage115   = data.voltage[0] / VOLTAGE_DIVIDER;
        float pressure115  = (voltage115 / 5.15 + 0.095) / 0.009;
        float voltage400   = data.voltage[1] / VOLTAGE_DIVIDER;
        float pressure400  = (voltage400 / 5.15 + 0.00842) / 0.002421;

        stats[0].add(pressure115);
        stats[1].add(pressure400);
        stats[2].add(data.voltage[2]);
        stats[3].add(data.voltage[3]);

        miosix::delayMs(50);
    }

    for (int i = 0; i < 4; i++)
    {
        StatsResult statsResults = stats[i].getStats();

        printf("%d: avg: %+.4f,\tmin: %+.4f,\tmax: %+.4f,\tstd: %.4f\n", i,
               statsResults.mean, statsResults.minValue, statsResults.maxValue,
               statsResults.stdDev);
    }

    miosix::delayMs(3000);

    // Sampling sensor
    printf("timestamp,Vpress115,Vpress400,VloadCell,Vbat,press115,press400\n");
    while (true)
    {
        ads131.sample();

        ADS131M04Data data = ads131.getLastSample();

        float voltage115  = data.voltage[0] / VOLTAGE_DIVIDER;
        float pressure115 = (voltage115 / 5.15 + 0.095) / 0.009;
        float voltage400  = data.voltage[1] / VOLTAGE_DIVIDER;
        float pressure400 = (voltage400 / 5.15 + 0.00842) / 0.002421;

        printf("[%f] % 2.5f,% 2.5f,% 2.5f,% 2.5f,% 2.5f,% 2.5f\n",
               data.timestamp / 1e6, data.voltage[0], data.voltage[1],
               data.voltage[2], data.voltage[3], pressure115, pressure400);

        delayMs(50);
    }
}
