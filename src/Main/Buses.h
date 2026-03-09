/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <drivers/spi/SPIDriver.h>
#include <drivers/usart/USART.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>

namespace Main
{

class Buses : public Boardcore::Injectable
{
public:
    Buses()
        : spi6(makeSPI(SPI6)), spi1(makeSPI(SPI1)), spi3(makeSPI(SPI3)),
          spi4(makeSPI(SPI4))
    {
    }

    Boardcore::SPIBusInterface& getH3LIS331DL() { return *spi1; }
    Boardcore::SPIBusInterface& getLPS22DF() { return *spi1; }
    Boardcore::SPIBusInterface& getLIS2MDL() { return *spi3; }
    Boardcore::SPIBusInterface& getLSM6DSRX() { return *spi3; }
    Boardcore::SPIBusInterface& getVN100() { return *spi1; }
    Boardcore::SPIBusInterface& getUBXGps() { return *spi3; }
    Boardcore::SPIBusInterface& getADS131M08() { return *spi4; }
    Boardcore::SPIBusInterface& getND015A() { return *spi4; }
    Boardcore::SPIBusInterface& getRadio() { return *spi6; }

    Boardcore::USART& getHILUart() { return usart4; }

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

        auto& dma = DMADriver::instance();

        auto txStream = dma.acquireStreamForPeripheral(txId, 1s);
        auto rxStream = dma.acquireStreamForPeripheral(rxId, 1s);

        if (!txStream.isValid() || !rxStream.isValid())
        {
            TRACE("Using CPU for SPI%d, streams unavailable: TX: %d, RX: %d\n",
                  spiNum, txStream.isValid(), rxStream.isValid());
            return std::make_unique<Boardcore::SPIBus>(spi);
        }

        TRACE("Using DMA for SPI%d\n", spiNum);
        return std::make_unique<Boardcore::SPIBusDMA>(spi, std::move(txStream),
                                                      std::move(rxStream));
    }

    // Initialized in this order to favor DMA to SPI6 (radio)
    std::unique_ptr<Boardcore::SPIBusInterface> spi6;
    std::unique_ptr<Boardcore::SPIBusInterface> spi1;
    std::unique_ptr<Boardcore::SPIBusInterface> spi3;
    std::unique_ptr<Boardcore::SPIBusInterface> spi4;

    Boardcore::USART usart4{UART4, 230400, 1024};
};

}  // namespace Main
