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

#include <Parafoil/ParafoilModule/ParafoilModule.h>
#include <drivers/spi/SPIBus.h>
#include <drivers/usart/USART.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

namespace Parafoil
{

struct Buses : public ParafoilModule
{
    Boardcore::USART hil;
    Boardcore::USART serialRadio;
    Boardcore::USART debug;
    Boardcore::USART uart4;

    Boardcore::SPIBus spi1;
    Boardcore::SPIBus spi2;

public:
#ifndef USE_MOCK_PERIPHERALS
    Buses()
        : hil(USART1, Boardcore::USARTInterface::Baudrate::B115200),
          serialRadio(USART2, Boardcore::USARTInterface::Baudrate::B115200),
          debug(USART3, Boardcore::USARTInterface::Baudrate::B115200),
          uart4(UART4, Boardcore::USARTInterface::Baudrate::B115200),
          spi1(SPI1), spi2(SPI2)
    {
        // miosix::interfaces::uart1::rx::getPin().mode(miosix::Mode::ALTERNATE);
        // miosix::interfaces::uart1::tx::getPin().mode(miosix::Mode::ALTERNATE);
        // miosix::interfaces::uart1::rx::getPin().alternateFunction(7);
        // miosix::interfaces::uart1::tx::getPin().alternateFunction(7);

        // miosix::interfaces::uart2::rx::getPin().mode(miosix::Mode::ALTERNATE);
        // miosix::interfaces::uart2::tx::getPin().mode(miosix::Mode::ALTERNATE);
        // miosix::interfaces::uart2::rx::getPin().alternateFunction(7);
        // miosix::interfaces::uart2::tx::getPin().alternateFunction(7);

        // hil.init();
        serialRadio.init();
        // debug.init();
        // uart4.init();
    }
#else
    Buses()
        : hil(USART1, Boardcore::USARTInterface::Baudrate::B115200),
          serialRadio(USART2, Boardcore::USARTInterface::Baudrate::B115200),
          debug(USART3, Boardcore::USARTInterface::Baudrate::B115200),
          uart4(UART4, Boardcore::USARTInterface::Baudrate::B115200), spi1({}),
          spi2({})
    {
        hil.init();
        serialRadio.init();
    }
#endif

    bool startModule() override { return true; }
};

}  // namespace Parafoil
