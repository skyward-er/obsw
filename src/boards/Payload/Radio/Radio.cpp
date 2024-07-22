/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/Radio/Radio.h>
#include <common/Radio.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace Boardcore;
using namespace Common;
namespace config = Payload::Config::Radio;

namespace
{

// Static radio instance that will handle the radio interrupts
SX1278Fsk* staticTransceiver = nullptr;

inline void handleDioIRQ()
{
    if (staticTransceiver)
    {
        staticTransceiver->handleDioIRQ();
    }
}

}  // namespace

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

namespace Payload
{

Radio::~Radio()
{
    if (staticTransceiver == transceiver.get())
    {
        staticTransceiver = nullptr;
    }
}

bool Radio::start()
{
    using namespace Units::Frequency;
    using namespace std::chrono;

    auto& scheduler = getModule<BoardScheduler>()->radio();

    // Initialize the radio
    auto frontend = std::make_unique<Skyward433Frontend>();

    transceiver = std::make_unique<SX1278Fsk>(
        getModule<Buses>()->radio(), miosix::radio::cs::getPin(),
        miosix::radio::dio0::getPin(), miosix::radio::dio1::getPin(),
        miosix::radio::dio3::getPin(), SPI::ClockDivider::DIV_128,
        std::move(frontend));

    // Set the static instance for handling radio interrupts
    staticTransceiver = transceiver.get();

    // Configure the radio
    if (transceiver->init(PAYLOAD_RADIO_CONFIG) != SX1278Fsk::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize the radio");
        return false;
    }

    // Initialize the Mavlink driver
    mavDriver = std::make_unique<MavDriver>(
        transceiver.get(),
        [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); },
        milliseconds{config::MavlinkDriver::SLEEP_AFTER_SEND}.count(),
        milliseconds{config::MavlinkDriver::MAX_PKT_AGE}.count());

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize the Mavlink driver");
        return false;
    }

    // Add the high rate telemetry task
    auto highRateTask = scheduler.addTask(
        [this]()
        {
            enqueueHighRateTelemetry();
            flushMessageQueue();
        },
        Config::Radio::HIGH_RATE_TELEMETRY);

    if (highRateTask == 0)
    {
        LOG_ERR(logger, "Failed to add the high rate telemetry task");
        return false;
    }

    auto lowRateTask = scheduler.addTask(
        [this]()
        {
            enqueueLowRateTelemetry();
            flushMessageQueue();
        },
        Config::Radio::LOW_RATE_TELEMETRY);

    if (lowRateTask == 0)
    {
        LOG_ERR(logger, "Failed to add the low rate telemetry task");
        return false;
    }

    started = true;
    return true;
}

bool Radio::isStarted() { return started; }

void Radio::enqueueAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(config::Mavlink::SYSTEM_ID,
                            config::Mavlink::COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueueMessage(ackMsg);
}

void Radio::enqueueNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(config::Mavlink::SYSTEM_ID,
                             config::Mavlink::COMPONENT_ID, &nackMsg, msg.msgid,
                             msg.seq, 0);
    enqueueMessage(nackMsg);
}

void Radio::enqueueHighRateTelemetry() { enqueueSystemTm(MAV_FLIGHT_ID); }

void Radio::enqueueLowRateTelemetry() { enqueueSystemTm(MAV_STATS_ID); }

void Radio::enqueueMessage(const mavlink_message_t& msg)
{
    Lock<FastMutex> lock(queueMutex);

    // Insert the message inside the queue only if there is enough space
    if (messageQueueIndex < messageQueue.size())
    {
        messageQueue[messageQueueIndex] = msg;
        messageQueueIndex++;
    }
}

void Radio::flushMessageQueue()
{
    Lock<FastMutex> lock(queueMutex);

    for (uint32_t i = 0; i < messageQueueIndex; i++)
    {
        mavDriver->enqueueMsg(messageQueue[i]);
    }

    // Reset the index
    messageQueueIndex = 0;
}

void Radio::logStatus() { Logger::getInstance().log(mavDriver->getStatus()); }

}  // namespace Payload
