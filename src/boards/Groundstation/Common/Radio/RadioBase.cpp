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

#include "RadioBase.h"

#include <Groundstation/Common/HubBase.h>

#include <memory>

using namespace miosix;
using namespace Groundstation;
using namespace Boardcore;

bool RadioBase::sendMsg(const mavlink_message_t& msg)
{
    Lock<FastMutex> l(pending_msgs_mutex);
    if (pending_msgs_count >= MAV_PENDING_OUT_QUEUE_SIZE)
    {
        return false;
    }
    else
    {
        pending_msgs[pending_msgs_count] = msg;
        pending_msgs_count += 1;

        return true;
    }
}

void RadioBase::handleDioIRQ()
{
    if (started)
    {
        sx1278->handleDioIRQ();
    }
}

RadioStats RadioBase::getStats()
{
    if (started)
    {
        auto mav_stats = mav_driver->getStatus();

        return {.send_errors = mav_stats.nSendErrors,
                .packet_rx_success_count =
                    mav_stats.mavStats.packet_rx_success_count,
                .packet_rx_drop_count = mav_stats.mavStats.packet_rx_drop_count,
                .bits_rx_count        = bits_rx_count,
                .bits_tx_count        = bits_tx_count,
                .rx_rssi              = sx1278->getLastRxRssi(),
                .rx_fei               = sx1278->getLastRxFei()};
    }
    else
    {
        return {0};
    }
}

bool RadioBase::start(std::unique_ptr<SX1278Fsk> sx1278)
{
    this->sx1278 = std::move(sx1278);

    auto mav_handler = [this](RadioMavDriver* channel,
                              const mavlink_message_t& msg) { handleMsg(msg); };

    mav_driver = std::make_unique<RadioMavDriver>(
        this, mav_handler, Groundstation::MAV_SLEEP_AFTER_SEND,
        Groundstation::MAV_OUT_BUFFER_MAX_AGE);

    if (!mav_driver->start())
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

void RadioBase::run()
{

    while (!shouldStop())
    {
        miosix::Thread::sleep(AUTOMATIC_FLUSH_PERIOD);

        // If enough time has passed, automatically flush.
        if (miosix::getTick() > last_eot_packet_ts + AUTOMATIC_FLUSH_DELAY)
        {
            flush();
        }
    }
}

ssize_t RadioBase::receive(uint8_t* pkt, size_t max_len)
{
    ssize_t ret = sx1278->receive(pkt, max_len);
    if (ret > 0)
    {
        bits_rx_count += ret * 8;
    }

    return ret;
}

bool RadioBase::send(uint8_t* pkt, size_t len)
{
    bool ret = sx1278->send(pkt, len);
    if (ret)
    {
        bits_tx_count += len * 8;
    }

    return ret;
}

void RadioBase::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    ModuleManager::getInstance().get<HubBase>()->dispatchIncomingMsg(msg);

    if (isEndOfTransmissionPacket(msg))
    {
        last_eot_packet_ts = miosix::getTick();
        flush();
    }
}

void RadioBase::flush()
{
    Lock<FastMutex> l(pending_msgs_mutex);
    for (size_t i = 0; i < pending_msgs_count; i++)
    {
        mav_driver->enqueueMsg(pending_msgs[i]);
    }

    pending_msgs_count = 0;
}

bool RadioBase::isEndOfTransmissionPacket(const mavlink_message_t& msg)
{
    return msg.msgid == MAVLINK_MSG_ID_ROCKET_FLIGHT_TM ||
           msg.msgid == MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM;
}