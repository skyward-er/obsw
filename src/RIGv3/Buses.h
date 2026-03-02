/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <drivers/dma/DMA.h>
#include <drivers/spi/SPIDriver.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>
#include <iostream>
#include <memory>

namespace RIGv3
{

class Buses : public Boardcore::Injectable
{
public:
    Buses() : spi2(makeSPI(SPI2)), spi3(makeSPI(SPI3)), spi4(makeSPI(SPI4)) {}

    Boardcore::SPIBusInterface& getADC1() { return *spi3; }
    Boardcore::SPIBusInterface& getADC2() { return *spi3; }
    Boardcore::SPIBusInterface& getADC3() { return *spi3; }
    Boardcore::SPIBusInterface& getADC4() { return *spi3; }
    Boardcore::SPIBusInterface& getRadio() { return *spi4; }

    miosix::GpioPin getADC1CsPin()
    {
        return miosix::interfaces::spi3::cs6::getPin();
    }
    miosix::GpioPin getADC2CsPin()
    {
        return miosix::interfaces::spi3::cs7::getPin();
    }
    miosix::GpioPin getADC3CsPin()
    {
        return miosix::interfaces::spi3::cs8::getPin();
    }
    miosix::GpioPin getADC4CsPin()
    {
        return miosix::interfaces::spi3::cs9::getPin();
    }

private:
    std::unique_ptr<Boardcore::SPIBusInterface> makeSPI(SPI_TypeDef* spi)
    {
        using namespace Boardcore;
        using namespace std::chrono;

        int spiNum = (spi == SPI1)   ? 1
                     : (spi == SPI2) ? 2
                     : (spi == SPI3) ? 3
                     : (spi == SPI4) ? 4
                     : (spi == SPI5) ? 5
                     : (spi == SPI6) ? 6
                                     : 0;
        auto txId  = (spi == SPI1)   ? DMADefs::Peripherals::PE_SPI1_TX
                     : (spi == SPI2) ? DMADefs::Peripherals::PE_SPI2_TX
                     : (spi == SPI3) ? DMADefs::Peripherals::PE_SPI3_TX
                     : (spi == SPI4) ? DMADefs::Peripherals::PE_SPI4_TX
                     : (spi == SPI5) ? DMADefs::Peripherals::PE_SPI5_TX
                     : (spi == SPI6) ? DMADefs::Peripherals::PE_SPI6_TX
                                     : DMADefs::Peripherals::PE_MEM_ONLY;
        auto rxId  = (spi == SPI1)   ? DMADefs::Peripherals::PE_SPI1_RX
                     : (spi == SPI2) ? DMADefs::Peripherals::PE_SPI2_RX
                     : (spi == SPI3) ? DMADefs::Peripherals::PE_SPI3_RX
                     : (spi == SPI4) ? DMADefs::Peripherals::PE_SPI4_RX
                     : (spi == SPI5) ? DMADefs::Peripherals::PE_SPI5_RX
                     : (spi == SPI6) ? DMADefs::Peripherals::PE_SPI6_RX
                                     : DMADefs::Peripherals::PE_MEM_ONLY;

        std::cout << "SPI" << spiNum << ": Acquiring DMA streams..."
                  << std::endl;

        auto& dma = DMADriver::instance();

        auto streamTx =
            DMAStreamGuard(dma.acquireStreamForPeripheral(txId, 1s));
        auto streamRx =
            DMAStreamGuard(dma.acquireStreamForPeripheral(rxId, 1s));

        if (!streamTx.isValid() || !streamRx.isValid())
        {
            std::cerr << "SPI" << spiNum
                      << ": Failed to acquire DMA streams (TX: "
                      << streamTx.isValid() << " RX: " << streamRx.isValid()
                      << "), falling back to CPU-only SPI bus" << std::endl;
            return std::make_unique<Boardcore::SPIBus>(spi);
        }

        std::cout << "SPI" << spiNum << ": Using DMA SPI bus" << std::endl;
        return std::make_unique<Boardcore::SPIBusDMA>(spi, std::move(streamTx),
                                                      std::move(streamRx));
    }

    std::unique_ptr<Boardcore::SPIBusInterface> spi2;
    std::unique_ptr<Boardcore::SPIBusInterface> spi3;
    std::unique_ptr<Boardcore::SPIBusInterface> spi4;
};

}  // namespace RIGv3
