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
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Ports/SerialLyraGS.h>

using namespace Groundstation;
using namespace GroundstationBase;
using namespace Boardcore;
using namespace LyraGS;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg)
{
    LyraGS::BoardStatus* status = getModule<LyraGS::BoardStatus>();

    bool send_ok = false;

    if (status->isMainRadioPresent() && msg.sysid == MAV_SYSID_MAIN)
    {
        LyraGS::RadioMain* radio = getModule<LyraGS::RadioMain>();
        send_ok |= radio->sendMsg(msg);
    }

    if (status->isPayloadRadioPresent() && msg.sysid == MAV_SYSID_PAYLOAD)
    {
        LyraGS::RadioPayload* radio = getModule<LyraGS::RadioPayload>();
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
    LyraGS::BoardStatus* status = getModule<LyraGS::BoardStatus>();

    SerialLyraGS* serial = getModule<SerialLyraGS>();
    serial->sendMsg(msg);

    if (status->isEthernetPresent())
    {
        LyraGS::EthernetGS* ethernet = getModule<LyraGS::EthernetGS>();
        ethernet->sendMsg(msg);
    }
}