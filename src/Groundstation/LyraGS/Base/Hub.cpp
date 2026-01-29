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

#include <Groundstation/Common/Config/EthernetConfig.h>
#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Ports/SerialLyraGS.h>
#include <common/Radio.h>

using namespace Groundstation;
using namespace GroundstationBase;
using namespace Boardcore;
using namespace LyraGS;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg)
{
    LyraGS::BoardStatus* status = getModule<LyraGS::BoardStatus>();

    // Handle GS_DISCOVERY_REQUEST (responds via both Serial and Ethernet)
    if (msg.msgid == MAVLINK_MSG_ID_GS_DISCOVERY_REQUEST)
    {
        mavlink_gs_discovery_response_t response;

        // Network info (0 if no ethernet)
        if (status->isEthernetPresent())
        {
            LyraGS::EthernetGS* ethernet = getModule<LyraGS::EthernetGS>();
            Boardcore::WizIp currentIp   = ethernet->getCurrentIp();
            response.ip                  = static_cast<uint32_t>(currentIp);
            response.port                = Groundstation::RECV_PORT;
        }
        else
        {
            response.ip   = 0;
            response.port = 0;
        }

        // Device and radio info (always included)
        response.device_id      = status->getSystemId();
        response.radio_433_type = status->getRadio433Type();
        response.radio_868_type = status->getRadio868Type();
        response.radio_433_frequency =
            (response.radio_433_type != RADIO_433_TYPE_NONE)
                ? Common::MAIN_RADIO_CONFIG.freq_rf
                : 0;
        response.radio_868_frequency =
            (response.radio_868_type != RADIO_868_TYPE_NONE)
                ? Common::PAYLOAD_RADIO_CONFIG.freq_rf
                : 0;

        // Encode and send the response
        mavlink_message_t responseMsg;
        mavlink_msg_gs_discovery_response_encode(
            Groundstation::GS_SYSTEM_ID, Groundstation::GS_COMPONENT_ID,
            &responseMsg, &response);

        dispatchIncomingMsg(responseMsg);
        return;  // Don't forward discovery requests to radios
    }

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
