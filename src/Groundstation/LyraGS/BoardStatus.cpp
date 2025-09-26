/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include <common/Radio.h>

#include <limits>

using namespace Boardcore;
using namespace Groundstation;
using namespace LyraGS;

static constexpr int8_t clampRssi(float rssi)
{
    constexpr float min = std::numeric_limits<int8_t>::min();
    constexpr float max = std::numeric_limits<int8_t>::max();

    return std::min(max, std::max(min, rssi));
}

bool BoardStatus::isMainRadioPresent() { return radio_433_type; }
bool BoardStatus::isPayloadRadioPresent() { return radio_868_type; }
bool BoardStatus::isEthernetPresent() { return ethernet_present; }

bool BoardStatus::start()
{
    if (!ActiveObject::start())
        return false;

    return true;
}

void BoardStatus::setRadio433Present(bool backup)
{
    radio_433_type = backup ? RADIO_433_TYPE_EBYTE : RADIO_433_TYPE_SKYWARD;
}

void BoardStatus::setRadio868Present(bool backup)
{
    radio_868_type = backup ? RADIO_868_TYPE_EBYTE : RADIO_868_TYPE_SKYWARD;
}

void BoardStatus::setEthernetPresent(bool present)
{
    ethernet_present = present;
}

void BoardStatus::sendArpTm()
{
    using namespace Antennas;

    auto vn300 = getModule<Sensors>()->getVN300LastSample();

    Actuators* actuators = getModule<Actuators>();
    SMA* sm              = getModule<SMA>();

    AntennaAngles targetAngles = sm->getTargetAngles();

    mavlink_arp_tm_t tm = {};
    tm.timestamp        = TimestampTimer::getTimestamp(); /*< [us] Timestamp*/
    tm.state            = static_cast<uint8_t>(sm->getStatus()); /*<  State*/
    tm.yaw              = vn300.yaw;          /*< [deg] Current Yaw*/
    tm.pitch            = vn300.pitch;        /*< [deg] Current Pitch*/
    tm.roll             = vn300.roll;         /*< [deg] Current Roll*/
    tm.target_yaw       = targetAngles.yaw;   /*< [deg] Target Yaw*/
    tm.target_pitch     = targetAngles.pitch; /*< [deg] Target Pitch*/
    tm.stepperX_pos     = actuators->getCurrentDegPosition(
        StepperList::STEPPER_X); /*< [deg] StepperX target pos*/
    tm.stepperX_delta = actuators->getDeltaAngleDeg(
        StepperList::STEPPER_X); /*< [deg] StepperX target delta deg*/
    tm.stepperX_speed =
        actuators->getSpeed(StepperList::STEPPER_X); /*< [rps] StepperX Speed*/
    tm.stepperY_pos = actuators->getCurrentDegPosition(
        StepperList::STEPPER_Y); /*< [deg] StepperY target pos*/
    tm.stepperY_delta = actuators->getDeltaAngleDeg(
        StepperList::STEPPER_Y); /*< [deg] StepperY target delta deg*/
    tm.stepperY_speed =
        actuators->getSpeed(StepperList::STEPPER_Y); /*< [rps] StepperY Speed*/
    tm.gps_latitude  = vn300.latitude;               /*< [deg] Latitude*/
    tm.gps_longitude = vn300.longitude;              /*< [deg] Longitude*/
    tm.gps_height    = vn300.altitude;               /*< [m] Altitude*/
    tm.gps_fix       = vn300.gpsFix; /*<  Wether the GPS has a FIX*/
    tm.log_number =
        Logger::getInstance().getCurrentLogNumber(); /*<  Log number*/

    tm.battery_voltage = -420.0;

    mavlink_message_t msg;
    mavlink_msg_arp_tm_encode(systemId, componentId, &msg, &tm);
    getModule<HubBase>()->dispatchIncomingMsg(msg);
}

void BoardStatus::sendRadioLinkTm()
{
    using namespace Groundstation;

    mavlink_rocket_radio_link_info_tm_t tm = {};
    tm.timestamp                           = TimestampTimer::getTimestamp();

    if (radio_433_type)
    {
        tm.radio_433_type = radio_433_type;
        tm.main_frequency = Common::MAIN_RADIO_CONFIG.freq_rf;

        auto stats               = getModule<RadioMain>()->getStats();
        tm.main_rx_success_count = stats.packet_rx_success_count;
        tm.main_rx_drop_count    = stats.packet_rx_drop_count;
        tm.main_bitrate          = main_rx_bitrate.update(stats.bits_rx_count);
        tm.main_rssi             = clampRssi(stats.rx_rssi);

        last_main_stats = stats;
    }

    if (radio_868_type)
    {
        tm.radio_868_type    = radio_868_type;
        tm.payload_frequency = Common::PAYLOAD_RADIO_CONFIG.freq_rf;

        auto stats                  = getModule<RadioPayload>()->getStats();
        tm.payload_rx_success_count = stats.packet_rx_success_count;
        tm.payload_rx_drop_count    = stats.packet_rx_drop_count;
        tm.payload_bitrate = payload_rx_bitrate.update(stats.bits_rx_count);
        tm.payload_rssi    = clampRssi(stats.rx_rssi);

        last_payload_stats = stats;
    }

    if (ethernet_present)
    {
        auto ethernet = getModule<EthernetGS>();
        auto stats    = ethernet->getState();

        tm.ethernet_status = (stats.link_up ? 1 : 0) |
                             (stats.full_duplex ? 2 : 0) |
                             (stats.based_100mbps ? 4 : 0);
    }

    mavlink_message_t msg;
    mavlink_msg_rocket_radio_link_info_tm_encode(systemId, componentId, &msg,
                                                 &tm);

    getModule<HubBase>()->dispatchIncomingMsg(msg);
}

void BoardStatus::run()
{
    while (!shouldStop())
    {
        if (isArp)
            sendArpTm();

        sendRadioLinkTm();

        miosix::Thread::sleep(RADIO_STATUS_PERIOD);
    }
}
