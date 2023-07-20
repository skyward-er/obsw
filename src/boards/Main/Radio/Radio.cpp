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
#include <Main/Buses.h>
#include <Main/Radio/Radio.h>

using namespace Boardcore;
namespace Main
{
Radio::Radio() {}

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Config the transceiver
    EbyteFsk::Config config;
    config.pa_boost = true;
    config.power    = 2;
    config.ocp      = 0;

    // Config the SPI
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.bitOrder     = SPI::Order::MSB_FIRST;
    spiConfig.writeBit     = SPI::WriteBit::INVERTED;

    transceiver = new EbyteFsk(SPISlave(modules.get<Buses>()->spi6,
                                        miosix::radio::cs::getPin(), spiConfig),
                               miosix::radio::tx_enable::getPin(),
                               miosix::radio::rx_enable::getPin());

    return true;
}

void Radio::sendAck(const mavlink_message_t& msg) {}

void Radio::sendNack(const mavlink_message_t& msg) {}

void Radio::logStatus() {}

void Radio::isStarted() {}

void Radio::handleMavlinkMessage(const mavlink_message_t& msg) {}

void Radio::handleCommand(const mavlink_message_t& msg) {}
}  // namespace Main