/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Raul Radu
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

#include "BoardStatus.h"

#include <ConRIGv2/Configs/HubConfig.h>
#include <common/Radio.h>

using namespace std::chrono;
using namespace ConRIGv2;

static constexpr int8_t clampTo8bits(float value)
{
    constexpr float min = std::numeric_limits<int8_t>::min();
    constexpr float max = std::numeric_limits<int8_t>::max();

    return std::min(max, std::max(min, value));
}

bool BoardStatus::start()
{
    if (!Boardcore::ActiveObject::start())
        return false;
    return true;
}

void BoardStatus::sendRadioLinkInfoTm()
{
    mavlink_gse_radio_link_info_tm_t tm = {};
    tm.timestamp = Boardcore::TimestampTimer::getTimestamp();

    auto radioStats     = getModule<Radio>()->getStats();
    tm.rssi             = clampTo8bits(radioStats.rssi);
    tm.snr              = clampTo8bits(radioStats.snr);
    tm.rx_success_count = radioStats.rxSuccessCount;
    tm.rx_drop_count    = radioStats.rxDropCount;
    tm.bitrate          = radioBitrate.update(radioStats.bitsRxCount);
    tm.frequency        = Common::RIG_RADIO_CONFIG.freq_rf;
    if (Config::Hub::ETHERNET_ENABLED)
    {
        auto ethState      = getModule<Hub>()->getEthernetState();
        tm.ethernet_status = (ethState.link_up ? 1 : 0) |
                             (ethState.full_duplex ? 2 : 0) |
                             (ethState.based_100mbps ? 4 : 0);
    }

    mavlink_message_t msg;
    mavlink_msg_gse_radio_link_info_tm_encode(
        Config::Hub::MAV_SYSTEM_ID, Config::Hub::MAV_COMPONENT_ID, &msg, &tm);

    getModule<Hub>()->dispatchToPorts(msg);
}

void BoardStatus::run()
{
    while (!shouldStop())
    {
        sendRadioLinkInfoTm();

        miosix::Thread::sleep(
            milliseconds{Config::Radio::RADIO_STATUS_PERIOD}.count());
    }
}
