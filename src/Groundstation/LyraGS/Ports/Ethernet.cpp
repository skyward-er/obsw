/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "Ethernet.h"

#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Buses.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace LyraGS;

namespace LyraGS
{
EthernetGS* ethernetGSGlobal = nullptr;
}

void __attribute__((used)) MIOSIX_ETHERNET_IRQ()
{
    if (ethernetGSGlobal)
        ethernetGSGlobal->handleINTn();
}

namespace LyraGS
{

bool EthernetGS::start()
{
    std::shared_ptr<Wiz5500> wiz5500 = std::make_shared<Wiz5500>(
        getModule<Buses>()->ethernet_bus, miosix::ethernet::cs::getPin(),
        miosix::ethernet::intr::getPin(), SPI::ClockDivider::DIV_64);

    // First check if the device is even connected
    bool present = wiz5500->checkVersion();

    if (!present)
        return false;

    if (!EthernetBase::start(wiz5500))
        return false;

    ethernetGSGlobal = this;
    getModule<BoardStatus>()->setEthernetPresent(true);

    return true;
}

void EthernetGS::sendMsg(const mavlink_message_t& msg) { Super::sendMsg(msg); };

void EthernetGS::handleINTn() { EthernetBase::handleINTn(); }
Boardcore::Wiz5500::PhyState EthernetGS::getState()
{
    return Super::getState();
};
}  // namespace LyraGS
