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

#include "RadioStatus.h"

#include <Groundstation/Base/Radio/Radio.h>
#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/Common/HubBase.h>
#include <common/Mavlink.h>
#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace Groundstation;
using namespace GroundstationBase;

bool RadioStatus::isMainRadioPresent() { return main_radio_present; }
bool RadioStatus::isPayloadRadioPresent() { return payload_radio_present; }

bool RadioStatus::start()
{
    if (!ActiveObject::start())
    {
        return false;
    }

    return true;
}

void RadioStatus::setMainRadioPresent(bool present)
{
    main_radio_present = present;
}

void RadioStatus::setPayloadRadioPresent(bool present)
{
    payload_radio_present = present;
}

void RadioStatus::run()
{
    while (!shouldStop())
    {
        miosix::Thread::sleep(RADIO_STATUS_PERIOD);

        mavlink_receiver_tm_t tm = {0};
        tm.timestamp             = TimestampTimer::getTimestamp();

        if (main_radio_present)
        {
            tm.main_radio_present = 1;

            auto stats =
                ModuleManager::getInstance().get<RadioMain>()->getStats();
            tm.main_packet_tx_error_count = stats.send_errors;
            tm.main_tx_bitrate = main_tx_bitrate.update(stats.bits_tx_count);
            tm.main_packet_rx_success_count = stats.packet_rx_success_count;
            tm.main_packet_rx_drop_count    = stats.packet_rx_drop_count;
            tm.main_rx_bitrate = main_rx_bitrate.update(stats.bits_rx_count);
            tm.main_rx_rssi    = stats.rx_rssi;
            tm.main_rx_fei     = stats.rx_fei;

            last_main_stats = stats;
        }

        if (payload_radio_present)
        {
            tm.payload_radio_present = 1;

            auto stats =
                ModuleManager::getInstance().get<RadioPayload>()->getStats();
            tm.payload_packet_tx_error_count = stats.send_errors;
            tm.payload_tx_bitrate =
                payload_tx_bitrate.update(stats.bits_tx_count);
            tm.payload_packet_rx_success_count = stats.packet_rx_success_count;
            tm.payload_packet_rx_drop_count    = stats.packet_rx_drop_count;
            tm.payload_rx_bitrate =
                payload_rx_bitrate.update(stats.bits_rx_count);
            tm.payload_rx_rssi = stats.rx_rssi;
            tm.payload_rx_fei  = stats.rx_fei;

            last_payload_stats = stats;
        }

        mavlink_message_t msg;
        mavlink_msg_receiver_tm_encode(GS_SYSTEM_ID, GS_COMPONENT_ID, &msg,
                                       &tm);

        ModuleManager::getInstance().get<HubBase>()->dispatchIncomingMsg(msg);
    }
}