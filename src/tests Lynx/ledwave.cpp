/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Luca Conterio
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

#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <stdio.h>

using led1 = miosix::leds::led_red1;
using led2 = miosix::leds::led_red2;
using led3 = miosix::leds::led_blue1;

/* NOTE:
 * These are conencted to the enable pin of the thermal
 * cutters and the cs of the lis3mdl magnetometer
 */
using led4 = miosix::leds::led_blue2;
using led5 = miosix::leds::led_green1;
using led6 = miosix::leds::led_green2;

// Test timeout
constexpr int TEST_TIMEOUT = 10;  // seconds

int main()
{
    printf("Waving!\n");

    // Sampling
    for (int i = 0; i < TEST_TIMEOUT; i++)
    {
        // On
        led3::high();
        miosix::delayMs(500 / 6);
        led1::high();
        miosix::delayMs(500 / 6);
        led5::high();
        miosix::delayMs(500 / 6);
        led6::high();
        miosix::delayMs(500 / 6);
        led2::high();
        miosix::delayMs(500 / 6);
        led4::high();
        miosix::delayMs(500 / 6);

        // Off
        led3::low();
        miosix::delayMs(500 / 6);
        led1::low();
        miosix::delayMs(500 / 6);
        led5::low();
        miosix::delayMs(500 / 6);
        led6::low();
        miosix::delayMs(500 / 6);
        led2::low();
        miosix::delayMs(500 / 6);
        led4::low();
        miosix::delayMs(500 / 6);
    }

    return 0;
}
