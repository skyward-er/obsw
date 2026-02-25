/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <interfaces-impl/hwmapping.h>
#include <drivers/spi/SPIBus.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv3
{

class Buses : public Boardcore::Injectable
{
public:
    Buses() : spi2(SPI2), spi3(SPI3), spi4(SPI4) {}

    Boardcore::SPIBus& getADC1() { return spi2; }
    Boardcore::SPIBus& getADC2() { return spi3; }
    Boardcore::SPIBus& getADC3() { return spi2; }
    Boardcore::SPIBus& getADC4() { return spi3; }
    Boardcore::SPIBus& getRadio() { return spi4; }

    miosix::GpioPin getADC1CsPin()
    {
        return miosix::interfaces::spi2::cs1::getPin();
    }
    miosix::GpioPin getADC2CsPin()
    {
        return miosix::interfaces::spi3::cs6::getPin();
    }
    miosix::GpioPin getADC3CsPin()
    {
        return miosix::interfaces::spi2::cs2::getPin();
    }
    miosix::GpioPin getADC4CsPin()
    {
        return miosix::interfaces::spi3::cs7::getPin();
    }

private:
    Boardcore::SPIBus spi2;
    Boardcore::SPIBus spi3;
    Boardcore::SPIBus spi4;
};

}  // namespace RIGv3
