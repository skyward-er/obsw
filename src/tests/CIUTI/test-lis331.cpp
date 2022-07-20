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

#include <miosix.h>
#include <sensors/LIS331HH/LIS331HH.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    SPIBus spi2(SPI2);
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_256;
    config.mode         = SPI::Mode::MODE_0;
    LIS331HH lis(spi2, sensors::lis331hh::cs::getPin(), config);

    lis.init();

    while (true)
    {
        lis.sample();
        auto data = lis.getLastSample();
        printf("[%.2f] %.3f %.3f %.3f\n", data.accelerationTimestamp / 1e6,
               data.accelerationX, data.accelerationY, data.accelerationZ);

        delayMs(250);
    }
}