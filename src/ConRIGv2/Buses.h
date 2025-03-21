/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Ettore Pane
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

#include <drivers/spi/SPIBus.h>
#include <drivers/usart/USART.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace ConRIGv2
{

struct Buses : public Boardcore::Injectable
{
private:
    Boardcore::SPIBus spi1{SPI1};
    Boardcore::SPIBus spi6{SPI6};

    Boardcore::USART usart2{USART2, 115200};
    Boardcore::USART uart4{UART4, 115200};

public:
    Boardcore::SPIBus& getRadio() { return spi6; }

    Boardcore::USART& getUsart2() { return usart2; }
    Boardcore::USART& getUsart4() { return uart4; }

    Boardcore::SPIBus& getEthernet() { return spi1; }
};

}  // namespace ConRIGv2
