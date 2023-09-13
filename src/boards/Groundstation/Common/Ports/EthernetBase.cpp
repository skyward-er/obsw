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

#include "EthernetBase.h"

#include <Groundstation/Common/Config/EthernetConfig.h>
#include <Groundstation/Common/HubBase.h>

#include <random>

using namespace Groundstation;
using namespace Boardcore;
using namespace miosix;

WizIp Groundstation::genNewRandomIp()
{
    WizIp ip = IP_BASE;
    ip.d     = (rand() % 253) + 1;  // Generate in range 1-254

    return ip;
}

WizMac Groundstation::genNewRandomMac()
{
    WizMac mac = MAC_BASE;
    mac.e      = (rand() % 253) + 1;  // Generate in range 1-254
    mac.f      = (rand() % 253) + 1;  // Generate in range 1-254

    return mac;
}

void EthernetBase::handleINTn()
{
    if (wiz5500)
    {
        wiz5500->handleINTn();
    }
}

void EthernetBase::sendMsg(const mavlink_message_t& msg)
{
    if (mav_driver && mav_driver->isStarted())
    {
        mav_driver->enqueueMsg(msg);
    }
}

bool EthernetBase::start(std::unique_ptr<Boardcore::Wiz5500> wiz5500)
{
    this->wiz5500 = std::move(wiz5500);

    // Reset the device
    if (!this->wiz5500->reset())
    {
        return false;
    }

    // Setup ip and other stuff
    this->wiz5500->setSubnetMask(SUBNET);
    this->wiz5500->setGatewayIp(GATEWAY);
    this->wiz5500->setSourceIp(genNewRandomIp());
    this->wiz5500->setSourceMac(genNewRandomMac());

    this->wiz5500->setOnIpConflict(
        [this]() { this->wiz5500->setSourceIp(genNewRandomIp()); });

    // Ok now open the UDP socket
    if (!this->wiz5500->openUdp(0, RECV_PORT, {255, 255, 255, 255}, SEND_PORT,
                                500))
    {
        return false;
    }

    auto mav_handler = [this](EthernetMavDriver* channel,
                              const mavlink_message_t& msg) { handleMsg(msg); };

    mav_driver = std::make_unique<EthernetMavDriver>(this, mav_handler, 0, 10);

    if (!mav_driver->start())
    {
        return false;
    }

    return true;
}

void EthernetBase::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    ModuleManager::getInstance().get<HubBase>()->dispatchOutgoingMsg(msg);
}

ssize_t EthernetBase::receive(uint8_t* pkt, size_t max_len)
{
    return wiz5500->recv(0, pkt, max_len);
}

bool EthernetBase::send(uint8_t* pkt, size_t len)
{
    return wiz5500->send(0, pkt, len, 100);
}