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

#include "Hub.h"

#include <Groundstation/Base/BoardStatus.h>
#include <Groundstation/Base/Ports/Ethernet.h>
#include <Groundstation/Base/Radio/Radio.h>
#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/Common/Ports/Serial.h>

using namespace Groundstation;
using namespace GroundstationBase;
using namespace Boardcore;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg)
{
    BoardStatus* status = ModuleManager::getInstance().get<BoardStatus>();

    bool send_ok = false;

    if (status->isMainRadioPresent() && msg.sysid == MAV_SYSID_MAIN)
    {
        RadioMain* radio = ModuleManager::getInstance().get<RadioMain>();
        send_ok |= radio->sendMsg(msg);
    }

    if (status->isPayloadRadioPresent() && msg.sysid == MAV_SYSID_PAYLOAD)
    {
        RadioPayload* radio = ModuleManager::getInstance().get<RadioPayload>();
        send_ok |= radio->sendMsg(msg);
    }

    (void)send_ok;

    // If both of the sends went wrong, just send a nack
    // This doesn't work well with multiple GS on the same ethernet network
    // if (!send_ok)
    // {
    //     sendNack(msg);
    // }
}

void Hub::dispatchIncomingMsg(const mavlink_message_t& msg)
{
    BoardStatus* status = ModuleManager::getInstance().get<BoardStatus>();

    Serial* serial = ModuleManager::getInstance().get<Serial>();
    serial->sendMsg(msg);

    if (status->isEthernetPresent())
    {
        Ethernet* ethernet = ModuleManager::getInstance().get<Ethernet>();
        ethernet->sendMsg(msg);
    }
}