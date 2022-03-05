/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#pragma once

#include <radio/Xbee/Xbee.h>

using namespace Boardcore;

namespace ParafoilTestDev
{
    //TODO change the pins to something correct
    static miosix::GpioPin XBEE_CS(GPIOC_BASE, 1);
    static miosix::GpioPin XBEE_ATTN(GPIOF_BASE, 10); //Interrupt pin (THE SAME AS THE RADIO.CPP)
    static miosix::GpioPin XBEE_RESET(GPIOC_BASE, 5);

    static miosix::GpioPin XBEE_SCK(GPIOE_BASE, 2);
	static miosix::GpioPin XBEE_MISO(GPIOE_BASE, 5);
	static miosix::GpioPin XBEE_MOSI(GPIOE_BASE, 6);

    //Data rate
    static const bool XBEE_80KBPS_DATA_RATE = true;
    static const int XBEE_TIMEOUT           = 5000; //5 seconds
}