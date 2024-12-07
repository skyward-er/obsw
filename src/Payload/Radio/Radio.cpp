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

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;
namespace config = Payload::Config::Radio;

namespace
{

// Static radio instance that will handle the radio interrupts
std::atomic<SX1278Fsk*> staticTransceiver{nullptr};

inline void handleDioIRQ()
{
    auto transceiver = staticTransceiver.load();
    if (transceiver)
        transceiver->handleDioIRQ();
}

}  // namespace

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

namespace Payload
{

Radio::~Radio()
{
    auto transceiverPtr = transceiver.get();
    staticTransceiver.compare_exchange_strong(transceiverPtr, nullptr);
}

bool Radio::start()
{
    auto& scheduler = getModule<BoardScheduler>()->radio();

    // Initialize the radio
    auto frontend = std::make_unique<Skyward433Frontend>();

    transceiver = std::make_unique<SX1278Fsk>(
        getModule<Buses>()->radio(), miosix::radio::cs::getPin(),
        miosix::radio::dio0::getPin(), miosix::radio::dio1::getPin(),
        miosix::radio::dio3::getPin(), SPI::ClockDivider::DIV_128,
        std::move(frontend));

    // Configure the radio
    if (transceiver->init(PAYLOAD_RADIO_CONFIG) != SX1278Fsk::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize the radio");
        return false;
    }

    // Set the static instance for handling radio interrupts
    staticTransceiver = transceiver.get();

    // Initialize the Mavlink driver
    radioMavlink.driver = std::make_unique<MavDriver>(
        transceiver.get(), [this](MavDriver*, const mavlink_message_t& msg)
        { handleRadioMessage(msg); },
        milliseconds{config::MavlinkDriver::SLEEP_AFTER_SEND}.count(),
        milliseconds{config::MavlinkDriver::MAX_PKT_AGE}.count());

    if (!radioMavlink.driver->start())
    {
        LOG_ERR(logger, "Failed to initialize the Mavlink driver");
        return false;
    }

    // Enable Mavlink over HIL USART only when HIL is not active
    if (config::MAVLINK_OVER_HIL_SERIAL_ENABLED &&
        !PersistentVars::getHilMode())
    {
        initMavlinkOverSerial();
    }

    // Add the high rate telemetry task
    auto highRateTask =
        scheduler.addTask([this]() { enqueueHighRateTelemetry(); },
                          Config::Radio::HIGH_RATE_TELEMETRY);

    if (highRateTask == 0)
    {
        LOG_ERR(logger, "Failed to add the high rate telemetry task");
        return false;
    }

    auto lowRateTask =
        scheduler.addTask([this]() { enqueueLowRateTelemetry(); },
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

void Radio::initMavlinkOverSerial()
{
    serialTransceiver =
        std::make_unique<SerialTransceiver>(getModule<Buses>()->HILUart());

    serialMavlink.driver = std::make_unique<MavDriver>(
        serialTransceiver.get(),
        [this](MavDriver*, const mavlink_message_t& msg)
        { handleSerialMessage(msg); },
        milliseconds{config::MavlinkDriver::SLEEP_AFTER_SEND}.count(),
        milliseconds{config::MavlinkDriver::MAX_PKT_AGE}.count());

    if (!serialMavlink.driver->start())
    {
        LOG_ERR(logger,
                "Failed to initialize mavlink driver over HIL serial, "
                "continuing without it");

        serialMavlink.driver.reset();
        serialTransceiver.reset();
    }
}

void Radio::handleRadioMessage(const mavlink_message_t& msg)
{
    radioMavlink.handleMessage(msg);
}

void Radio::handleSerialMessage(const mavlink_message_t& msg)
{
    serialMavlink.handleMessage(msg);
    serialMavlink.flushQueue();
}

void Radio::enqueueHighRateTelemetry()
{
    radioMavlink.enqueueSystemTm(MAV_FLIGHT_ID);
    radioMavlink.flushQueue();
}

void Radio::enqueueLowRateTelemetry()
{
    radioMavlink.enqueueSystemTm(MAV_STATS_ID);
}

void Radio::MavlinkBackend::enqueueMessage(const mavlink_message_t& msg)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);

    // Insert the message inside the queue only if there is enough space
    if (index < queue.size())
    {
        queue[index] = msg;
        index++;
    }
}

void Radio::MavlinkBackend::flushQueue()
{
    Lock<FastMutex> lock(mutex);

    for (uint32_t i = 0; i < index; i++)
        driver->enqueueMsg(queue[i]);

    // Reset the index
    index = 0;
}

void Radio::MavlinkBackend::logStatus()
{
    Logger::getInstance().log(driver->getStatus());
}

}  // namespace Payload
