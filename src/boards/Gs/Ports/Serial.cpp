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

#include "Serial.h"

#include <Gs/Radio/Radio.h>
#include <Gs/Radio/RadioStatus.h>

using namespace miosix;
using namespace Gs;
using namespace Boardcore;

bool Serial::start()
{
    ActiveObject::start();

    return true;
}

void Serial::sendMsg(const mavlink_message_t &msg)
{
    Lock<FastMutex> l(mutex);
    uint8_t msg_buf[MAVLINK_NUM_NON_PAYLOAD_BYTES +
                    MAVLINK_MAX_DIALECT_PAYLOAD_SIZE];
    int msg_len = mavlink_msg_to_send_buffer(msg_buf, &msg);

    auto serial = miosix::DefaultConsole::instance().get();
    serial->writeBlock(msg_buf, msg_len, 0);
}

void Serial::handleMsg(const mavlink_message_t &msg)
{
    // TODO:
    RadioStatus *status = ModuleManager::getInstance().get<RadioStatus>();

    if (status->isMainRadioPresent())
    {
        RadioMain *radio = ModuleManager::getInstance().get<RadioMain>();
        radio->sendMsg(msg);
    }

    if (status->isPayloadRadioPresent())
    {
        RadioPayload *radio = ModuleManager::getInstance().get<RadioPayload>();
        radio->sendMsg(msg);
    }
}

void Serial::run()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t msg_buf[256];

    while (!shouldStop())
    {
        auto serial = miosix::DefaultConsole::instance().get();
        int rcv_len = serial->readBlock(msg_buf, sizeof(msg_buf), 0);

        for (int i = 0; i < rcv_len; i++)
        {
            uint8_t parse_result =
                mavlink_parse_char(MAVLINK_COMM_0, msg_buf[i], &msg, &status);

            if (parse_result == 1)
            {
                handleMsg(msg);
            }
        }
    }
}