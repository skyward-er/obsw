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

#pragma once

#include <common/MavlinkOrion.h>
#include <drivers/WIZ5500/WIZ5500.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <radio/UdpTransceiver/UdpTransceiver.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>

namespace ConRIGv2
{
class Buses;
class Radio;

using SerialMavDriver =
    Boardcore::MavlinkDriver<1024, 10, MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;
using EthernetMavDriver =
    Boardcore::MavlinkDriver<1024, 10, MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

/**
 * @brief Central hub connecting all outgoing and incoming modules.
 */
class Hub : public Boardcore::InjectableWithDeps<Buses, Radio>
{
public:
    bool start();

    /**
     * @brief Dispatch messages to external ports (serial, ethernet).
     */
    void dispatchToPorts(const mavlink_message_t& msg);

    /**
     * @brief Dispatch messages to the RIG.
     */
    void dispatchToRIG(const mavlink_message_t& msg);

    Boardcore::Wiz5500::PhyState getEthernetState();

    Boardcore::WizIp getCurrentIp() { return currentIp; }
    Boardcore::WizMac getCurrentMac() { return currentMac; }

    void printIpConfig(std::ostream& os) const;

private:
    bool initEthernet();

    std::unique_ptr<Boardcore::SerialTransceiver> serial2;
    std::unique_ptr<SerialMavDriver> mavDriver2;

    std::unique_ptr<Boardcore::SerialTransceiver> serial4;
    std::unique_ptr<SerialMavDriver> mavDriver4;

    std::shared_ptr<Boardcore::Wiz5500> wiz5500;
    std::unique_ptr<Boardcore::UdpTransceiver> ethernet;
    std::unique_ptr<EthernetMavDriver> mavDriverEth;
    Boardcore::WizIp currentIp   = {};
    Boardcore::WizMac currentMac = {};

    bool started = false;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Hub");
};
}  // namespace ConRIGv2
