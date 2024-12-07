/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include "Serial.h"

#include <ConRIG/Radio/Radio.h>
#include <filesystem/console/console_device.h>

using namespace miosix;
using namespace ConRIG;
using namespace Boardcore;

bool Serial::start()
{
    mavDriver = std::make_unique<SerialMavDriver>(
        this, [this](SerialMavDriver* channel, const mavlink_message_t& msg)
        { handleMessage(msg); }, 0, 10);

    if (!mavDriver->start())
        return false;

    return true;
}

void Serial::sendMessage(const mavlink_message_t& msg)
{
    if (mavDriver)
        mavDriver->enqueueMsg(msg);
}

void Serial::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nack;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nack, msg.msgid,
                             msg.seq, 100);
    sendMessage(nack);
}

void Serial::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ack;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ack, msg.msgid,
                            msg.seq);
    sendMessage(ack);
}

void Serial::handleMessage(const mavlink_message_t& msg)
{
    if (!getModule<Radio>()->enqueueMessage(msg))
        sendNack(msg);
}

ssize_t Serial::receive(uint8_t* pkt, size_t maxLen)
{
    auto serial = miosix::DefaultConsole::instance().get();
    return serial->readBlock(pkt, maxLen, 0);
}

bool Serial::send(uint8_t* pkt, size_t len)
{
    auto serial = miosix::DefaultConsole::instance().get();
    return serial->writeBlock(pkt, len, 0) != static_cast<ssize_t>(len);
}
