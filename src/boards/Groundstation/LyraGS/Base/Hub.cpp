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

#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Radio/Radio.h>

using namespace Groundstation;
using namespace GroundstationBase;
using namespace Boardcore;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg)
{
    LyraGS::BoardStatus* status =
        ModuleManager::getInstance().get<LyraGS::BoardStatus>();

    bool send_ok = false;

    if (status->isMainRadioPresent() && msg.sysid == MAV_SYSID_MAIN)
    {
        LyraGS::RadioMain* radio =
            ModuleManager::getInstance().get<LyraGS::RadioMain>();
        send_ok |= radio->sendMsg(msg);
    }

    if (status->isPayloadRadioPresent() && msg.sysid == MAV_SYSID_PAYLOAD)
    {
        LyraGS::RadioPayload* radio =
            ModuleManager::getInstance().get<LyraGS::RadioPayload>();
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
    LyraGS::BoardStatus* status =
        ModuleManager::getInstance().get<LyraGS::BoardStatus>();

    Serial* serial = ModuleManager::getInstance().get<Serial>();
    serial->sendMsg(msg);

    if (status->isEthernetPresent())
    {
        LyraGS::Ethernet* ethernet =
            ModuleManager::getInstance().get<LyraGS::Ethernet>();
        ethernet->sendMsg(msg);
    }
}