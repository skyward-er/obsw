/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Nicolò Caruso
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

#include "EthernetUtils.h"

using namespace Groundstation;
using namespace Boardcore;
using namespace miosix;

void EthernetBase::handleINTn()
{
    if (wiz5500)
        wiz5500->handleINTn();
}

void EthernetBase::sendMsg(const mavlink_message_t& msg)
{
    if (mav_driver && mav_driver->isStarted())
        mav_driver->enqueueMsg(msg);
}

Boardcore::Wiz5500::PhyState EthernetBase::getState()
{
    return wiz5500->getPhyState();
}

void EthernetBase::printIpConfig(std::ostream& os) const
{
    os << "Ethernet state:"
       << "\n\tIP address:  " << currentIp
       << "\n\tSubnet mask: " << Groundstation::SUBNET
       << "\n\tGateway:     " << Groundstation::GATEWAY
       << "\n\tMAC address: " << currentMac << std::endl;
}

bool EthernetBase::start(std::shared_ptr<Boardcore::Wiz5500> wiz5500)
{
    this->wiz5500 = wiz5500;

    // Reset the device
    this->wiz5500->reset();

    // Setup ip and other stuff
    this->wiz5500->setSubnetMask(SUBNET);
    this->wiz5500->setGatewayIp(GATEWAY);

    // Set with the dipswitch offset
    if (!randomIp)
    {
        WizIp ip = GS_IP_BASE;
        ip.d = 1 + ipOffset;  // Add to the ip the offset set on the dipswitch
        currentIp = ip;
        this->wiz5500->setSourceIp(ip);

        WizMac mac = GS_MAC_BASE;
        // Add to the mac address the offset set on the dipswitch
        mac.c += 1 + ipOffset;
        // In case of sniffing change ulteriorly the MAC to avoid switch to
        // filter based on not whole MAC...
        if (sniffOtherGs)
        {
            mac.a += 1;
            mac.b += 1;
            mac.c += 1;
            mac.d += 1;
            mac.e += 1;
            mac.f += 1;
        }
        currentMac = mac;
        this->wiz5500->setSourceMac(mac);
    }
    else
    {
        currentIp  = generateRandomIpAddress(IP_BASE, SUBNET);
        currentMac = generateRandomMacAddress(MAC_BASE);
        this->wiz5500->setSourceIp(currentIp);
        this->wiz5500->setSourceMac(currentMac);
    }

    this->wiz5500->setOnIpConflict(
        [this]()
        {
            currentIp = generateRandomIpAddress(IP_BASE, SUBNET);
            this->wiz5500->setSourceIp(currentIp);

            // Print the new configuration
            std::cout << "Ethernet IP configuration changed\n";
            this->printIpConfig(std::cout);
        });

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
        return false;

    // In case of sniffing mode initialize and start the EthernetSniffer
    if (sniffOtherGs)
    {
        getModule<EthernetSniffer>()->init(1, SEND_PORT, RECV_PORT);
        if (!getModule<EthernetSniffer>()->start(this->wiz5500))
            return false;
    }

    TRACE("[info] Ethernet module started correctly\n");
    return true;
}

void EthernetBase::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    getModule<HubBase>()->dispatchOutgoingMsg(msg);
}

ssize_t EthernetBase::receive(uint8_t* pkt, size_t max_len)
{
    WizIp dst_ip;
    uint16_t dst_port;
    return wiz5500->recvfrom(0, pkt, max_len, dst_ip, dst_port);
}

bool EthernetBase::send(uint8_t* pkt, size_t len)
{
    return wiz5500->send(0, pkt, len, 1000);
    // return true;
}
