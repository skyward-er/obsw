/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Pietro Bortolus
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
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv3
{

class Buses : public Boardcore::Injectable
{
public:
    Buses()
        : i2c1(I2C1, miosix::interfaces::i2c1::scl::getPin(),
               miosix::interfaces::i2c1::sda::getPin()),
          spi2(SPI2), spi3(SPI3), spi4(SPI4)
    {
    }

    Boardcore::SPIBus& getADC0() { return spi2; }
    Boardcore::SPIBus& getADC1() { return spi2; }
    Boardcore::SPIBus& getADC2() { return spi2; }
    Boardcore::SPIBus& getADC3() { return spi2; }
    Boardcore::SPIBus& getExpander() { return spi3; }
    Boardcore::SPIBus& getRadio() { return spi4; }
    Boardcore::I2C& getPCA9685() { return i2c1; }

    miosix::GpioPin getADC0CsPin()
    {
        return miosix::interfaces::spi2::cs1::getPin();
    }
    miosix::GpioPin getADC1CsPin()
    {
        return miosix::interfaces::spi2::cs2::getPin();
    }
    miosix::GpioPin getADC2CsPin()
    {
        return miosix::interfaces::spi2::cs3::getPin();
    }
    miosix::GpioPin getADC3CsPin()
    {
        return miosix::interfaces::spi2::cs4::getPin();
    }

private:
    Boardcore::I2C i2c1;
    Boardcore::SPIBus spi2;
    Boardcore::SPIBus spi3;
    Boardcore::SPIBus spi4;
};

}  // namespace RIGv3
