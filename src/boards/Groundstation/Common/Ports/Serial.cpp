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

#include <Groundstation/Common/HubBase.h>
#include <filesystem/console/console_device.h>

using namespace miosix;
using namespace Groundstation;
using namespace Boardcore;

bool Serial::start()
{
    auto mav_handler = [this](SerialMavDriver* channel,
                              const mavlink_message_t& msg) { handleMsg(msg); };

    mav_driver = std::make_unique<SerialMavDriver>(this, mav_handler, 0, 10);

    if (!mav_driver->start())
    {
        return false;
    }

    return true;
}

void Serial::sendMsg(const mavlink_message_t& msg)
{
    if (mav_driver && mav_driver->isStarted())
    {
        mav_driver->enqueueMsg(msg);
    }
}

void Serial::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    ModuleManager::getInstance().get<HubBase>()->dispatchOutgoingMsg(msg);
}

ssize_t Serial::receive(uint8_t* pkt, size_t max_len)
{
    auto serial = miosix::DefaultConsole::instance().get();
    return serial->readBlock(pkt, max_len, 0);
}

bool Serial::send(uint8_t* pkt, size_t len)
{
    auto serial = miosix::DefaultConsole::instance().get();
    return serial->writeBlock(pkt, len, 0) == static_cast<ssize_t>(len);
}