/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Davide Basso
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

#include "Radio.h"

#include <MockupMain/BoardScheduler.h>
#include <MockupMain/Buses.h>
#include <common/Radio.h>
#include <radio/Xbee/APIFramesLog.h>
#include <radio/Xbee/ATCommands.h>

using namespace Boardcore;
using namespace Boardcore::Units::Time;
using namespace Common;
namespace config = MockupMain::Config::Radio;

namespace
{

// Static radio instance that will handle the radio interrupts
std::atomic<Xbee::Xbee*> staticTransceiver{nullptr};
inline void handleDioIRQ()
{
    auto transceiver = staticTransceiver.load();
    if (transceiver)
        transceiver->handleATTNInterrupt();
}

}  // namespace

void __attribute__((used)) EXTI1_IRQHandlerImpl() { handleDioIRQ(); }

namespace MockupMain
{

Radio::~Radio()
{
    auto transceiverPtr = transceiver.get();
    staticTransceiver.compare_exchange_strong(transceiverPtr, nullptr);
}

bool Radio::start()
{
    auto& scheduler = getModule<BoardScheduler>()->radio();

    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    transceiver = std::make_unique<Xbee::Xbee>(
        getModule<Buses>()->spi2, config, miosix::xbee::cs::getPin(),
        miosix::xbee::attn::getPin(), miosix::xbee::reset::getPin());
    transceiver->setOnFrameReceivedListener([this](Xbee::APIFrame& frame)
                                            { handleXbeeFrame(frame); });

    Xbee::setDataRate(*transceiver, Config::Radio::Xbee::ENABLE_80KPS_DATA_RATE,
                      Millisecond{Config::Radio::Xbee::TIMEOUT}.value());

    // Set the static instance for handling radio interrupts
    staticTransceiver = transceiver.get();

    // Initialize the Mavlink driver
    radioMavlink.driver = std::make_unique<MavDriver>(
        transceiver.get(), [this](MavDriver*, const mavlink_message_t& msg)
        { handleRadioMessage(msg); },
        Millisecond{config::MavlinkDriver::SLEEP_AFTER_SEND}.value(),
        Millisecond{config::MavlinkDriver::MAX_PKT_AGE}.value());

    if (!radioMavlink.driver->start())
    {
        LOG_ERR(logger, "Failed to initialize the Mavlink driver");
        return false;
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

void Radio::handleXbeeFrame(Boardcore::Xbee::APIFrame& frame)
{
    using namespace Xbee;
    bool logged = false;
    switch (frame.frameType)
    {
        case FTYPE_AT_COMMAND:
        {
            ATCommandFrameLog dest;
            logged = ATCommandFrameLog::toFrameType(frame, &dest);
            if (logged)
                Logger::getInstance().log(dest);
            break;
        }
        case FTYPE_AT_COMMAND_RESPONSE:
        {
            ATCommandResponseFrameLog dest;
            logged = ATCommandResponseFrameLog::toFrameType(frame, &dest);
            if (logged)
                Logger::getInstance().log(dest);
            break;
        }
        case FTYPE_MODEM_STATUS:
        {
            ModemStatusFrameLog dest;
            logged = ModemStatusFrameLog::toFrameType(frame, &dest);
            if (logged)
                Logger::getInstance().log(dest);
            break;
        }
        case FTYPE_TX_REQUEST:
        {
            TXRequestFrameLog dest;
            logged = TXRequestFrameLog::toFrameType(frame, &dest);
            if (logged)
                Logger::getInstance().log(dest);
            break;
        }
        case FTYPE_TX_STATUS:
        {
            TXStatusFrameLog dest;
            logged = TXStatusFrameLog::toFrameType(frame, &dest);
            if (logged)
                Logger::getInstance().log(dest);
            break;
        }
        case FTYPE_RX_PACKET_FRAME:
        {
            RXPacketFrameLog dest;
            logged = RXPacketFrameLog::toFrameType(frame, &dest);
            if (logged)
                Logger::getInstance().log(dest);
            break;
        }
    }

    if (!logged)
    {
        APIFrameLog api;
        APIFrameLog::fromAPIFrame(frame, &api);
        Logger::getInstance().log(api);
    }
}

void Radio::handleRadioMessage(const mavlink_message_t& msg)
{
    radioMavlink.handleMessage(msg);
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

}  // namespace MockupMain
