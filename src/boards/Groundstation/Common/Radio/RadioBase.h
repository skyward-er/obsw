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
#include <Groundstation/Common/Config/RadioConfig.h>
#include <common/Mavlink.h>
#include <common/Radio.h>
#include <radio/MavlinkDriver/MavlinkDriverPigna.h>
#include <radio/SX1278/SX1278Fsk.h>

#include <memory>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Groundstation
{

using RadioMavDriver = Boardcore::MavlinkDriverPignaSlave<
    Boardcore::SX1278Fsk::MTU, Groundstation::MAV_PENDING_OUT_QUEUE_SIZE,
    MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

/**
 * @brief Statistics of the radio.
 */
struct RadioStats
{
    uint16_t send_errors;              //< Number of failed sends.
    uint16_t packet_rx_success_count;  //< Number of received packets.
    uint16_t packet_rx_drop_count;     //< Number of packet drops.
    uint32_t bits_rx_count;            //< Number of bits received.
    uint32_t bits_tx_count;            //< Number of bits sent.
    float rx_rssi;                     //< RSSI in dBm of last received packet.
    float rx_fei;  //< Frequency error index in Hz of last received packet.
};

/**
 * @brief Base radio class, used to implement functionality independent of
 * main/payload radios.
 */
class RadioBase : private Boardcore::ActiveObject, public Boardcore::Transceiver
{
public:
    RadioBase(uint8_t pingMsgId): pingMsgId(pingMsgId) {}

    /**
     * @brief Send a mavlink message through this radio.
     *
     * @returns false when the queue is full.
     */
    bool sendMsg(const mavlink_message_t& msg);

    /**
     * @brief Handle generic DIO irq.
     */
    void handleDioIRQ();

    /**
     * @brief Retrieve current statistics metrics.
     */
    RadioStats getStats();

protected:
    /**
     * @brief Initialize this radio module.
     */
    bool start(std::unique_ptr<Boardcore::SX1278Fsk> sx1278);

private:
    void run() override;

    ssize_t receive(uint8_t* pkt, size_t max_len) override;

    bool send(uint8_t* pkt, size_t len) override;

    bool started = false;

    uint32_t bits_rx_count = 0;
    uint32_t bits_tx_count = 0;

    // Message ID of the periodic message
    uint8_t pingMsgId;

    // Objects are always destructed in reverse order, so keep them in this
    // order
    std::unique_ptr<Boardcore::SX1278Fsk> sx1278;
    std::unique_ptr<RadioMavDriver> mavDriver;
};

}  // namespace Groundstation