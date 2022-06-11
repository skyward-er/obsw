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
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <sensors/ADS131M04/ADS131M04.h>

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
