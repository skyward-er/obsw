/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <con_RIG/BoardScheduler.h>
#include <con_RIG/Buses.h>
#include <con_RIG/Buttons/Buttons.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/Xbee/ATCommands.h>
#include <con_RIG/Buttons/Buttons.h>
#include <thread>

using namespace std;
using namespace miosix;
using namespace placeholders;
using namespace Boardcore;
using namespace con_RIG::RadioConfig;

namespace con_RIG
{

Mutex usart_mutex;

void Radio::handleMavlinkMessage(MavDriver* driver,
                                 const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        // case MAVLINK_MSG_ID_GSE_TM:
        case MAVLINK_MSG_ID_FSM_TM:
        {
            LOG_DEBUG(logger, "Received gse telemetry");
            mavlinkWriteToUsart(msg);
        }
        // case MAVLINK_MSG_ID_MOTOR_TM:
        case MAVLINK_MSG_ID_LOAD_TM:
        {
            LOG_DEBUG(logger, "Received motor telemetry");
            mavlinkWriteToUsart(msg);
        }
        default:
        {
            LOG_DEBUG(logger, "Received message is not of a known type");
            sendNack(msg);
            return;
        }
    }
    // Acknowledge the message
    sendAck(msg);
}

void Radio::mavlinkWriteToUsart(const mavlink_message_t& msg)
{
    Lock<Mutex> lock(usart_mutex);

    uint8_t temp_buf[MAVLINK_NUM_NON_PAYLOAD_BYTES +
                     MAVLINK_MAX_DIALECT_PAYLOAD_SIZE];
    int len = mavlink_msg_to_send_buffer(temp_buf, &msg);
    ModuleManager::getInstance().get<Buses>()->usart1.write(temp_buf, len);
}

void Radio::sendMessages()
{

    ButtonsState state = ModuleManager::getInstance().get<Buttons>()->getState();
    if (!state.empty()){
        // messages.push_back(state.buttonsToMavlink());    
    }
    if (messages.empty()){
        mavlink_message_t msg;
        mavlink_ping_tc_t ping;
        ping.timestamp = TimestampTimer::getTimestamp();
        mavlink_msg_ping_tc_encode(RadioConfig::MAV_SYSTEM_ID,
                                RadioConfig::MAV_COMPONENT_ID, &msg, &ping); 
        messages.push_back(msg);       
    }
    for (auto & message : messages) {
        mavDriver->enqueueMsg(message);
    }

    // TODO: syncronize with loopReadFromUsart
    messages.clear();
}

void Radio::loopReadFromUsart()
{
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = 0; // TODO: what is this?
    uint8_t byte;

    while (true)
    {
        int len = ModuleManager::getInstance().get<Buses>()->usart1.read(&byte, 1);
        if (len && mavlink_parse_char(chan, byte, &msg, &status)){
            // TODO: syncronize with sendMessages
            messages.push_back(msg);
        }
    }
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    mavDriver->enqueueMsg(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    LOG_DEBUG(logger, "Sending NACK for message {}", msg.msgid);
    mavlink_msg_nack_tm_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    mavDriver->enqueueMsg(nackMsg);
}

bool Radio::start() { 
    std::thread read([&]() { loopReadFromUsart(); });
    return mavDriver->start(); 
}

bool Radio::isStarted() { return mavDriver->isStarted(); }

Boardcore::MavlinkStatus Radio::getMavlinkStatus()
{
    return mavDriver->getStatus();
}

void Radio::logStatus()
{
    Logger::getInstance().log(mavDriver->getStatus());
    // TODO: Add transceiver status logging
}

Radio::Radio()
{

    ModuleManager& modules = ModuleManager::getInstance();

    // TODO: change to new radio
    transceiver =
        new SX1278(modules.get<Buses>()->spi2, Gpio<GPIOC_BASE, 1>::getPin());

    mavDriver = new MavDriver(transceiver,
                              bind(&Radio::handleMavlinkMessage, this, _1, _2),
                              0, MAV_OUT_BUFFER_MAX_AGE);

    modules.get<BoardScheduler>()->getScheduler().addTask(
        [&]() { sendMessages(); }, PING_GSE_PERIOD, PING_GSE_TASK_ID);
}

}  // namespace con_RIG
