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

#pragma once

#include <Singleton.h>
#include <drivers/spi/SPIBus.h>
#include <drivers/usart/USART.h>
#include <miosix.h>

namespace WindGallery
{

struct Buses : public Boardcore::Singleton<Buses>
{
    friend class Boardcore::Singleton<Buses>;

    Boardcore::USART uart4;
    Boardcore::SPIBus spi1;
    Boardcore::SPIBus spi2;

private:
#ifndef USE_MOCK_PERIPHERALS
    Buses()
        : uart4(UART4, Boardcore::USARTInterface::Baudrate::B115200),
          spi1(SPI1), spi2(SPI2)
    {
        uart4.init();
    }
#else
    Buses()
        : uart4(UART4, Boardcore::USARTInterface::Baudrate::B115200), spi1({}),
          spi2({})
    {
    }
#endif
};

}  // namespace WindGallery
