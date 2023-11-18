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

#include <Groundstation/Base/Buses.h>
#include <interfaces-impl/hwmapping.h>

using namespace Groundstation;
using namespace GroundstationBase;
using namespace Boardcore;
using namespace miosix;

void __attribute__((used)) MIOSIX_ETHERNET_IRQ()
{
    ModuleManager::getInstance().get<Ethernet>()->handleINTn();
}

bool Ethernet::start()
{
    std::unique_ptr<Wiz5500> wiz5500 = std::make_unique<Wiz5500>(
        ModuleManager::getInstance().get<Buses>()->ethernet_bus,
        ethernet::cs::getPin(), ethernet::intr::getPin(),
        SPI::ClockDivider::DIV_64);

    return EthernetBase::start(std::move(wiz5500));
}