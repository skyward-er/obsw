/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include "EthernetDiscovery.h"

#include <Groundstation/Common/Config/EthernetConfig.h>
#include <Groundstation/Common/HubBase.h>
#include <diagnostic/PrintLogger.h>

using namespace Groundstation;
using namespace Boardcore;

void EthernetDiscovery::init(uint16_t socketNr, uint16_t listenPort)
{
    this->socketNr   = socketNr;
    this->listenPort = listenPort;
}

bool EthernetDiscovery::start(std::shared_ptr<Boardcore::Wiz5500> wiz5500)
{
    this->wiz5500 = wiz5500;

    TRACE("[info] Opening discovery UDP socket on port %d\n", listenPort);

    // Open UDP socket for discovery channel
    if (!this->wiz5500->openUdp(socketNr, listenPort, {255, 255, 255, 255},
                                SEND_PORT, 500))
    {
        return false;
    }

    auto mav_handler = [this](EthernetMavDriver* channel,
                              const mavlink_message_t& msg) { handleMsg(msg); };

    mav_driver = std::make_unique<EthernetMavDriver>(this, mav_handler, 0, 10);

    if (!mav_driver->start())
        return false;

    TRACE("[info] EthernetDiscovery started on socket %d, port %d\n", socketNr,
          listenPort);

    return true;
}

void EthernetDiscovery::sendMsg(const mavlink_message_t& msg)
{
    if (mav_driver && mav_driver->isStarted())
        mav_driver->enqueueMsg(msg);
}

void EthernetDiscovery::handleMsg(const mavlink_message_t& msg)
{
    // Only handle discovery requests on this channel
    if (msg.msgid == MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST)
        getModule<HubBase>()->dispatchOutgoingMsg(msg);
    // All other messages are ignored on the discovery channel
}

ssize_t EthernetDiscovery::receive(uint8_t* pkt, size_t max_len)
{
    WizIp dst_ip;
    uint16_t dst_port;
    return wiz5500->recvfrom(socketNr, pkt, max_len, dst_ip, dst_port);
}

bool EthernetDiscovery::send(uint8_t* pkt, size_t len)
{
    return wiz5500->send(socketNr, pkt, len, 1000);
}
