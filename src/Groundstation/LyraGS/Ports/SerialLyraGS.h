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

#pragma once

#include <ActiveObject.h>
#include <Groundstation/Common/Ports/EthernetBase.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Buses.h>
#include <common/MavlinkLyra.h>
#include <drivers/usart/USART.h>
#include <filesystem/console/console_device.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>

namespace LyraGS
{

using SerialMavDriver =
    Boardcore::MavlinkDriver<1024, 10, MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

/**
 * @brief Class responsible for UART communication.
 */
class SerialLyraGS : public Boardcore::InjectableWithDeps<
                         Boardcore::InjectableBase<Groundstation::Serial>,
                         Buses, Groundstation::HubBase>
{
public:
    SerialLyraGS() {}

    /**
     * @brief Initialize the serial module.
     */
    [[nodiscard]] bool start();

    /**
     * @brief Send a mavlink message through this port.
     */
    void sendMsg(const mavlink_message_t& msg);

private:
    /**
     * @brief Called internally when a message is received.
     */
    void handleMsg(const mavlink_message_t& msg);

    ssize_t receive(uint8_t* pkt, size_t max_len) override;

    bool send(uint8_t* pkt, size_t len) override;

    miosix::FastMutex mutex;
    std::unique_ptr<SerialMavDriver> mav_driver;
};

}  // namespace LyraGS