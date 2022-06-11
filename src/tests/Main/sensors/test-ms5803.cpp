/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Nuno Barcellos
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
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorSampler.h>

using namespace Boardcore;
using namespace miosix;

/**
 * This test is intended to be run on the Death Stack X
 */

int main()
{
    // Sample temperature every 10 pressure samples
    SPIBus spiBus(SPI2);
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;
    MS5803 ms5803(spiBus, miosix::sensors::ms5803::cs::getPin(), {}, 10);

    Thread::sleep(100);

    if (!ms5803.init())
        printf("Init failed\n");

    Thread::sleep(100);

    while (true)
    {
        ms5803.sample();

        MS5803Data data = ms5803.getLastSample();
        printf("[%f] %f %f\n", data.pressureTimestamp / 1e6, data.pressure,
               data.temperature);

        Thread::sleep(20);
    }
}
