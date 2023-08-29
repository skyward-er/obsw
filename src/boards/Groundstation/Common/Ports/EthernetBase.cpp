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

#include <random>

#include <Groundstation/Common/HubBase.h>
#include <Groundstation/Common/Config/EthernetConfig.h>

#include "EthernetBase.h"

using namespace Groundstation;
using namespace Boardcore;
using namespace miosix;

WizIp Groundstation::genNewRandomIp() {
    WizIp ip = IP_BASE;
    ip.d = (rand() % 253) + 1; // Generate in range 1-254

    return ip;
}

WizMac Groundstation::genNewRandomMac() {
    WizMac mac = MAC_BASE;
    mac.e = (rand() % 253) + 1; // Generate in range 1-254
    mac.f = (rand() % 253) + 1; // Generate in range 1-254

    return mac;
}

void EthernetBase::handleINTn()
{
    if (started)
    {
        wiz5500->handleINTn();
    }
}

void EthernetBase::sendMsg(const mavlink_message_t& msg)
{
    if(started) {
        uint8_t msg_buf[MAVLINK_NUM_NON_PAYLOAD_BYTES +
                        MAVLINK_MAX_DIALECT_PAYLOAD_SIZE];
        int msg_len = mavlink_msg_to_send_buffer(msg_buf, &msg);
        wiz5500->send(0, msg_buf, msg_len, 100);
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

    this->wiz5500->setOnIpConflict([this]() {
        this->wiz5500->setSourceIp(genNewRandomIp());
    });

    // Ok now open the UDP socket
    if (!this->wiz5500->openUdp(0, RECV_PORT, {255, 255, 255, 255}, SEND_PORT, 500))
    {
        return false;
    }

    if (!ActiveObject::start())
    {
        return false;
    }

    started = true;
    return true;
}

void EthernetBase::run()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t msg_buf[256];

    while (!shouldStop())
    {
        WizIp dst_ip;
        uint16_t dst_port;
        ssize_t rcv_len = this->wiz5500->recvfrom(0, msg_buf, sizeof(msg_buf),
                                                  dst_ip, dst_port, 500);

        if (rcv_len == -1)
        {
            // Avoid spin looping, failure are highly likely to happen in
            // sequence
            Thread::sleep(100);
        }
        else
        {
            for (ssize_t i = 0; i < rcv_len; i++)
            {
                uint8_t parse_result = mavlink_parse_char(
                    MAVLINK_COMM_0, msg_buf[i], &msg, &status);

                if (parse_result == 1)
                {
                    // Dispatch the message through the hub.
                    ModuleManager::getInstance()
                        .get<HubBase>()
                        ->dispatchOutgoingMsg(msg);
                }
            }
        }
    }
}