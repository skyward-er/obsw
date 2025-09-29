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

#pragma once

#include <ActiveObject.h>
#include <Groundstation/Common/HubBase.h>
#include <common/MavlinkOrion.h>
#include <drivers/WIZ5500/WIZ5500.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>

namespace Groundstation
{

using EthernetMavDriver =
    Boardcore::MavlinkDriver<1024, 10, MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

class EthernetSniffer : public Boardcore::Transceiver,
                        public Boardcore::InjectableWithDeps<HubBase>
{
public:
    void handleINTn();

    Boardcore::Wiz5500::PhyState getState();

    bool start(std::shared_ptr<Boardcore::Wiz5500> wiz5500);

    void init(uint16_t portNumber, uint16_t srcPort, uint16_t dstPort);

private:
    std::shared_ptr<Boardcore::Wiz5500> wiz5500;

    /**
     * @brief Called internally when a message is received.
     */
    void handleMsg(const mavlink_message_t& msg);

    ssize_t receive(uint8_t* pkt, size_t max_len) override;

    bool send(uint8_t* pkt, size_t len) override;

    bool started = false;
    std::unique_ptr<EthernetMavDriver> mav_driver;
    uint16_t portNr;
    uint16_t srcPort;
    uint16_t dstPort;
    uint32_t sniffedCounter = 0;
};

}  // namespace Groundstation
