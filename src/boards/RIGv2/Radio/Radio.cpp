/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Buses.h>
#include <common/Radio.h>
#include <radio/SX1278/SX1278Frontends.h>

#include <atomic>

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

Boardcore::SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    Boardcore::SX1278Lora* instance = gRadio;
    if (instance)
    {
        instance->handleDioIRQ();
    }
}

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txEn::getPin(),
                                        radio::rxEn::getPin());

    // Setup transceiver
    radio = std::make_unique<SX1278Lora>(
        modules.get<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    {
        miosix::FastDisableInterrupt di;
        gRadio = radio.get();
    }

    // Initialize radio
    auto result = radio->init(RIG_RADIO_CONFIG);
    if (result != SX1278Lora::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize RIG radio");
        return false;
    }

    // Initialize mavdriver
    mavDriver = std::make_unique<MavDriver>(
        radio.get(),
        [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); },
        Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize RIG mav driver");
        return false;
    }

    return true;
}

void Radio::stop()
{
    // Remove global radio instance
    {
        miosix::FastDisableInterrupt di;
        gRadio = nullptr;
    }
    mavDriver->stop();
}

void Radio::queuePacket(const mavlink_message_t& msg)
{
    queuedPackets.put(msg);
}

void Radio::flushPackets()
{
    // Flush all packets of the queue
    size_t count = queuedPackets.count();
    for (size_t i = 0; i < count; i++)
    {
        try
        {
            mavDriver->enqueueMsg(queuedPackets.pop());
        }
        catch (...)
        {
            // This shouldn't happen, but still try to prevent it
        }
    }
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    queuePacket(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    queuePacket(nackMsg);
}

Boardcore::MavlinkStatus Radio::getMavStatus()
{
    return mavDriver->getStatus();
}

void Radio::handleMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
            sendAck(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_TC:
            handleCommand(msg);
            break;

        case MAVLINK_MSG_ID_CONRIG_STATE_TC:
            handleConrigState(msg);
            break;

        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
            // TODO: Implement SystemTM
            sendNack(msg);
            break;

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
            // TODO: Implement servo wiggle
            sendNack(msg);
            break;

        case MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC:
            // TODO: Implement set atomic valve timing
            sendNack(msg);
            break;

        case MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC:
            // TODO: Implement set valve maximum aperture
            sendNack(msg);
            break;

        case MAVLINK_MSG_ID_SET_IGNITION_TIME_TC:
            // TODO: Implement set ignition time
            sendNack(msg);
            break;

        default:
            // Unrecognized packet
            sendNack(msg);
            break;
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    uint8_t cmd = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmd)
    {
        case MAV_CMD_START_LOGGING:
            if (!Logger::getInstance().start())
            {
                sendNack(msg);
            }
            else
            {
                sendAck(msg);
            }
            break;

        case MAV_CMD_STOP_LOGGING:
            Logger::getInstance().stop();
            sendAck(msg);
            break;

        case MAV_CMD_CALIBRATE:
            // TODO: Implement calibrate
            sendNack(msg);
            break;

        default:
            // Unrecognized command
            sendNack(msg);
            break;
    }
}

void Radio::handleConrigState(const mavlink_message_t& msg)
{
    sendAck(msg);

    // TODO: Send actual data
    sendFakeGseTm();

    flushPackets();

    // TODO: Handle buttons
}

void Radio::sendFakeGseTm()
{
    mavlink_gse_tm_t tm = {0};
    mavlink_message_t msg;

    tm.timestamp = 69;

    mavlink_msg_gse_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                              Config::Radio::MAV_COMPONENT_ID, &msg, &tm);

    queuePacket(msg);
}
