/* Copyright (c) 2025 Skyward Experimental Rocketry
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

#include <Groundstation/Common/Config/EthernetConfig.h>
#include <Groundstation/Common/HubBase.h>

#include <random>

#include "EthernetBase.h"

using namespace Groundstation;
using namespace Boardcore;
using namespace miosix;

void EthernetSniffer::handleINTn()
{
    if (wiz5500)
        wiz5500->handleINTn();
}

bool EthernetSniffer::send(uint8_t* pkt, size_t len)
{
    // Send is not needed in sniffing, therefore not implemented
    return false;
};

Boardcore::Wiz5500::PhyState EthernetSniffer::getState()
{
    return wiz5500->getPhyState();
}

void EthernetSniffer::init(uint16_t portNumber, uint16_t srcPort,
                           uint16_t dstPort)
{
    portNr  = portNumber;
    srcPort = srcPort;
    dstPort = dstPort;
}

bool EthernetSniffer::start(std::shared_ptr<Boardcore::Wiz5500> wiz5500)
{
    this->wiz5500 = wiz5500;

    TRACE("[info] Opening sniffing UDP socket\n");
    // We open the UDP socket for sniffing
    if (!this->wiz5500->openUdp(1, SEND_PORT, {255, 255, 255, 255}, RECV_PORT,
                                500))
    {
        return false;
    }

    auto mav_handler = [this](EthernetMavDriver* channel,
                              const mavlink_message_t& msg) { handleMsg(msg); };

    mav_driver = std::make_unique<EthernetMavDriver>(this, mav_handler, 0, 10);

    if (!mav_driver->start())
        return false;

    TRACE("[info] EthernetSniffer start ok\n");

    return true;
}

void EthernetSniffer::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    getModule<HubBase>()->dispatchOutgoingMsg(msg);
}

ssize_t EthernetSniffer::receive(uint8_t* pkt, size_t max_len)
{
    WizIp dst_ip;
    uint16_t dst_port;
    return wiz5500->recvfrom(portNr, pkt, max_len, dst_ip, dst_port);
}
