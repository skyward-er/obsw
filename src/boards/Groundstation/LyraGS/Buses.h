/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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
#include <utils/DependencyManager/DependencyManager.h>

#include "interfaces-impl/hwmapping.h"

namespace LyraGS
{

class Buses : public Boardcore::Injectable
{
public:
    Boardcore::SPIBus &getRadio() { return radio1_bus; }

    Boardcore::SPIBus radio1_bus{MIOSIX_RADIO1_SPI};
    Boardcore::SPIBus radio2_bus{MIOSIX_RADIO2_SPI};
    Boardcore::USART usart2{USART2, 115200};
    Boardcore::USART uart4{UART4, 115200};
    Boardcore::SPIBus ethernet_bus{MIOSIX_ETHERNET_SPI};
};

}  // namespace LyraGS