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
#include <ConRIGv2/Configs/HubConfig.h>
#include <ConRIGv2/Radio/Radio.h>
#include <Groundstation/Common/Config/EthernetConfig.h>
#include <Groundstation/Common/Ports/EthernetUtils.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace Boardcore;
using namespace Groundstation;

static std::weak_ptr<Wiz5500> gEthernet;

void __attribute__((used)) MIOSIX_ETHERNET_IRQ()
{
    if (auto ethernet = gEthernet.lock())
        ethernet->handleINTn();
}

namespace ConRIGv2
{

Boardcore::Wiz5500::PhyState Hub::getEthernetState()
{
    return wiz5500->getPhyState();
}

bool Hub::start()
{
    auto* buses = getModule<Buses>();

    if (Config::Hub::USART2_ENABLED)
    {
        serial2 = std::make_unique<SerialTransceiver>(buses->getUsart2());

        mavDriver2 = std::make_unique<SerialMavDriver>(
            serial2.get(), [this](auto channel, const mavlink_message_t& msg)
            { dispatchToRIG(msg); }, 0, 10);

        if (!mavDriver2->start())
        {
            LOG_ERR(logger, "Error starting the MAVLink driver for USART2");
            return false;
        }
    }
    else
    {
        LOG_WARN(logger,
                 "Skipping USART2 initialization as it is disabled in the "
                 "configuration");
    }

    if (Config::Hub::USART4_ENABLED)
    {
        serial4 = std::make_unique<SerialTransceiver>(buses->getUsart4());

        mavDriver4 = std::make_unique<SerialMavDriver>(
            serial4.get(), [this](auto channel, const mavlink_message_t& msg)
            { dispatchToRIG(msg); }, 0, 10);

        if (!mavDriver4->start())
        {
            LOG_ERR(logger, "Error starting the MAVLink driver for USART4");
            return false;
        }
    }
    else
    {
        LOG_WARN(logger,
                 "Skipping USART4 initialization as it is disabled in the "
                 "configuration");
    }

    if (Config::Hub::ETHERNET_ENABLED)
    {
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
        {
            LOG_ERR(logger, "Error starting MAVLink driver for Ethernet");
            return false;
        }
    }
    else
    {
        LOG_WARN(logger,
                 "Skipping Ethernet initialization as it is disabled in the "
                 "configuration");
    }

    return true;
}

bool Hub::initEthernet()
{
    auto* buses = getModule<Buses>();

    // Initialize the Wiz5500
    auto wiz = std::make_unique<Wiz5500>(
        buses->getEthernet(), ethernet::cs::getPin(), ethernet::intr::getPin(),
        SPI::ClockDivider::DIV_64);

    // Check SPI communication
    if (!wiz->checkVersion())
    {
        LOG_ERR(logger, "Error checking the Wiz5500 version");
        return false;
    }

    // Store the Wiz5500 instance after checking device presence
    wiz5500 = std::move(wiz);

    // Store the global ethernet instance for the interrupt handler
    gEthernet = wiz5500;

    // Reset the device
    wiz5500->reset();

    // Setup ip and other stuff
    wiz5500->setSubnetMask(SUBNET);
    wiz5500->setGatewayIp(GATEWAY);

    currentIp  = generateRandomIpAddress(IP_BASE, SUBNET);
    currentMac = generateRandomMacAddress(MAC_BASE);
    wiz5500->setSourceIp(currentIp);
    wiz5500->setSourceMac(currentMac);

    wiz5500->setOnIpConflict(
        [this]
        {
            currentIp = generateRandomIpAddress(IP_BASE, SUBNET);
            wiz5500->setSourceIp(currentIp);

            // Print the new configuration
            std::cout << "Ethernet IP configuration changed\n";
            this->printIpConfig(std::cout);
        });

    // Ok now open the UDP socket
    if (!wiz5500->openUdp(0, RECV_PORT, {255, 255, 255, 255}, SEND_PORT, 500))
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

void Hub::printIpConfig(std::ostream& os) const
{
    os << "Ethernet state:"
       << "\n\tIP address:  " << currentIp
       << "\n\tSubnet mask: " << Groundstation::SUBNET
       << "\n\tGateway:     " << Groundstation::GATEWAY
       << "\n\tMAC address: " << currentMac << std::endl;
}

}  // namespace ConRIGv2
