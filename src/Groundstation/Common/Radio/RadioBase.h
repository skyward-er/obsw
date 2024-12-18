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
#include <Groundstation/Common/HubBase.h>
#include <common/MavlinkLyra.h>
#include <common/Radio.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <memory>

namespace Groundstation
{

using RadioMavDriver =
    Boardcore::MavlinkDriver<Boardcore::SX1278Fsk::MTU,
                             Groundstation::MAV_OUT_QUEUE_SIZE,
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
class RadioBase : private Boardcore::ActiveObject,
                  public Boardcore::Transceiver,
                  public Boardcore::InjectableWithDeps<HubBase>
{
public:
    RadioBase() {}

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

    /**
     * @brief Called internally when a message is received.
     */
    void handleMsg(const mavlink_message_t& msg);

    /**
     * @brief Flush all pending messages.
     */
    void flush();

    /**
     * @brief Check if a message signals an end of transmission
     */
    bool isEndOfTransmissionPacket(const mavlink_message_t& msg);

    bool started = false;

    miosix::FastMutex pending_msgs_mutex;
    mavlink_message_t pending_msgs[Groundstation::MAV_PENDING_OUT_QUEUE_SIZE];
    size_t pending_msgs_count = 0;

    long long last_eot_packet_ts = 0;

    uint32_t bits_rx_count = 0;
    uint32_t bits_tx_count = 0;

    // Objects are always destructed in reverse order, so keep them in this
    // order
    std::unique_ptr<Boardcore::SX1278Fsk> sx1278;
    std::unique_ptr<RadioMavDriver> mav_driver;
};

}  // namespace Groundstation
