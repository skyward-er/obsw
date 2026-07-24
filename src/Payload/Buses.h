/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <drivers/i2c/I2C.h>
#include <drivers/spi/SPIBus.h>
#include <drivers/usart/USART.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{

/**
 * @brief Manages all the buses of the payload board.
 * It provides access to the buses used by the sensors and other peripherals.
 */
class Buses : public Boardcore::Injectable
{
public:
    Buses() {}

    Boardcore::SPIBus& getH3LIS331DL() { return spi1; }
    Boardcore::SPIBus& getLPS22DF() { return spi1; }
    Boardcore::SPIBus& getLIS2MDL() { return spi3; }
    Boardcore::SPIBus& getLSM6DSRX() { return spi3; }
    Boardcore::SPIBus& getUBXGps() { return spi3; }
    Boardcore::SPIBus& getADS131M08() { return spi4; }
    Boardcore::SPIBus& getND015X() { return spi4; }
    Boardcore::SPIBus& radio() { return spi6; }

    Boardcore::USART& HILUart() { return usart4; }

private:
    Boardcore::SPIBus spi1{SPI1};
    Boardcore::SPIBus spi3{SPI3};
    Boardcore::SPIBus spi4{SPI4};
    Boardcore::SPIBus spi6{SPI6};

    Boardcore::USART usart4{UART4, 230400, 1024};
};
}  // namespace Payload
