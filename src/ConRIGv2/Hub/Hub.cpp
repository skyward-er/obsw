/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Ettore Pane
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

#include "Hub.h"

#include <ConRIGv2/Buses.h>
#include <ConRIGv2/Radio/Radio.h>
#include <Groundstation/Common/Config/EthernetConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace miosix;

static std::weak_ptr<Wiz5500> gEthernet;

void __attribute__((used)) MIOSIX_ETHERNET_IRQ()
{
    if (auto ethernet = gEthernet.lock())
        ethernet->handleINTn();
}

namespace ConRIGv2
{
WizIp generateRandomIp()
{
    WizIp ip = Groundstation::IP_BASE;
    ip.d     = (rand() % 253) + 1;  // Generate in range 1-254

    return ip;
}

WizMac generateRandomMac()
{
    WizMac mac = Groundstation::MAC_BASE;
    mac.e      = (rand() % 253) + 1;  // Generate in range 1-254
    mac.f      = (rand() % 253) + 1;  // Generate in range 1-254

    return mac;
}

Boardcore::Wiz5500::PhyState Hub::getEthernetState()
{
    return wiz5500->getPhyState();
}

bool Hub::start()
{
    auto* buses = getModule<Buses>();

    serial2 = std::make_unique<SerialTransceiver>(buses->getUsart2());
    serial4 = std::make_unique<SerialTransceiver>(buses->getUsart4());

    mavDriver2 = std::make_unique<SerialMavDriver>(
        serial2.get(), [this](auto channel, const mavlink_message_t& msg)
        { dispatchToRIG(msg); }, 0, 10);

    mavDriver4 = std::make_unique<SerialMavDriver>(
        serial4.get(), [this](auto channel, const mavlink_message_t& msg)
        { dispatchToRIG(msg); }, 0, 10);

    if (!mavDriver2->start())
    {
        LOG_ERR(logger, "Error starting the MAVLink driver for USART2");
        return false;
    }

    if (!mavDriver4->start())
    {
        LOG_ERR(logger, "Error starting the MAVLink driver for USART4");
        return false;
    }

    if (!initEthernet())
    {
        LOG_ERR(logger, "Error starting the Ethernet driver");
        return false;
    }

    mavDriverEth = std::make_unique<EthernetMavDriver>(
        ethernet.get(),
        [this](EthernetMavDriver* channel, const mavlink_message_t& msg)
        { dispatchToRIG(msg); }, 0, 10);

    if (!mavDriverEth->start())
        return false;

    return true;
}

bool Hub::initEthernet()
{
    auto* buses = getModule<Buses>();

    // Initialize the Wiz5500
    wiz5500 = std::make_unique<Wiz5500>(
        buses->getEthernet(), ethernet::cs::getPin(), ethernet::intr::getPin(),
        SPI::ClockDivider::DIV_64);

    // Store the global ethernet instance for the interrupt handler
    gEthernet = wiz5500;

    // Check SPI communication
    if (!wiz5500->checkVersion())
    {
        LOG_ERR(logger, "Error checking the Wiz5500 version");
        return false;
    }

    // Reset the device
    wiz5500->reset();

    // Setup ip and other stuff
    wiz5500->setSubnetMask(Groundstation::SUBNET);
    wiz5500->setGatewayIp(Groundstation::GATEWAY);
    wiz5500->setSourceIp(generateRandomIp());
    wiz5500->setSourceMac(generateRandomMac());

    wiz5500->setOnIpConflict([this]()
                             { wiz5500->setSourceIp(generateRandomIp()); });

    // Ok now open the UDP socket
    if (!wiz5500->openUdp(0, Groundstation::RECV_PORT, {255, 255, 255, 255},
                          Groundstation::SEND_PORT, 500))
    {
        LOG_ERR(logger, "Error opening the UDP socket");
        return false;
    }

    // Initialize Ethernet Transceiver
    ethernet = std::make_unique<UdpTransceiver>(this->wiz5500, 0);

    return true;
}

void Hub::dispatchToPorts(const mavlink_message_t& msg)
{
    if (mavDriver2 && mavDriver2->isStarted())
        mavDriver2->enqueueMsg(msg);

    if (mavDriver4 && mavDriver4->isStarted())
        mavDriver4->enqueueMsg(msg);

    if (mavDriverEth && mavDriverEth->isStarted())
        mavDriverEth->enqueueMsg(msg);
}

void Hub::dispatchToRIG(const mavlink_message_t& msg)
{
    if (msg.sysid == MAV_SYSID_RIG)
        getModule<Radio>()->enqueueMessage(msg);
}
}  // namespace ConRIGv2
