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

#include <Gs/Config/RadioConfig.h>
#include <common/Mavlink.h>
#include <common/Radio.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <utils/collections/CircularBuffer.h>
#include <ActiveObject.h>

#include <memory>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Gs
{

using MavDriver =
    Boardcore::MavlinkDriver<Boardcore::SX1278Fsk::MTU, Gs::MAV_OUT_QUEUE_SIZE,
                             MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

/**
 * @brief Base radio class, used to implement functionality independent of
 * main/payload radios.
 */
class RadioBase : private Boardcore::ActiveObject
{
public:
    RadioBase() {}

    /**
     * @brief Send a mavlink message through this radio.
     */
    void sendMsg(const mavlink_message_t& msg);

    /**
     * @brief Handle generic DIO irq.
     */
    void handleDioIRQ();

protected:
    /**
     * @brief Initialize this radio module.
     *
     * Used in RadioMain/RadioPayload to initialize the correct device with the
     * correct config.
     */
    bool start(std::unique_ptr<Boardcore::SX1278Fsk> sx1278,
               const Boardcore::SX1278Fsk::Config& config);

    void run() override;

private:
    /**
     * @brief Called internally when a message is received.
     */
    void handleMsg(const mavlink_message_t& msg);

    /**
     * @brief Flush all pending messages.
     */
    void flush();

    /**
     * @brief Check if a message signals an end of trasmissiont
     */
    bool isEndOfTransmissionPacket(const mavlink_message_t& msg);

    miosix::FastMutex mutex;

    Boardcore::CircularBuffer<mavlink_message_t, Gs::MAV_PENDING_OUT_QUEUE_SIZE>
        pending_msgs;

    long long last_eot_packet_ts = 0;

    // Objects are always destructed in reverse order, so keep them in this
    // order
    std::unique_ptr<Boardcore::SX1278Fsk> sx1278;
    std::unique_ptr<MavDriver> mav_driver;
};

class RadioMain : public RadioBase, public Boardcore::Module
{
public:
    [[nodiscard]] bool start();
};

class RadioPayload : public RadioBase, public Boardcore::Module
{
public:
    [[nodiscard]] bool start();
};

}  // namespace Gs