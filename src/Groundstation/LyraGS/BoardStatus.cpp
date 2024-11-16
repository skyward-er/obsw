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

using namespace Boardcore;
using namespace Groundstation;
using namespace LyraGS;

bool BoardStatus::isMainRadioPresent() { return main_radio_present; }
bool BoardStatus::isPayloadRadioPresent() { return payload_radio_present; }
bool BoardStatus::isEthernetPresent() { return ethernet_present; }

bool BoardStatus::start()
{
    if (!ActiveObject::start())
    {
        return false;
    }

    return true;
}

void BoardStatus::setMainRadioPresent(bool present)
{
    main_radio_present = present;
}

void BoardStatus::setPayloadRadioPresent(bool present)
{
    payload_radio_present = present;
}

void BoardStatus::setEthernetPresent(bool present)
{
    ethernet_present = present;
}

void BoardStatus::arpRoutine()
{
    using namespace Antennas;

    miosix::Thread::sleep(Groundstation::RADIO_STATUS_PERIOD);

    auto vn300 = getModule<Sensors>()->getVN300LastSample();

    Actuators *actuators = getModule<Actuators>();
    SMA *sm              = getModule<SMA>();

    AntennaAngles targetAngles = sm->getTargetAngles();

    mavlink_arp_tm_t tm = {0};
    tm.timestamp        = TimestampTimer::getTimestamp(); /*< [us] Timestamp*/
    tm.state        = static_cast<uint8_t>(sm->getStatus().state); /*<  State*/
    tm.yaw          = vn300.yaw;          /*< [deg] Current Yaw*/
    tm.pitch        = vn300.pitch;        /*< [deg] Current Pitch*/
    tm.roll         = vn300.roll;         /*< [deg] Current Roll*/
    tm.target_yaw   = targetAngles.yaw;   /*< [deg] Target Yaw*/
    tm.target_pitch = targetAngles.pitch; /*< [deg] Target Pitch*/
    tm.stepperX_pos = actuators->getCurrentDegPosition(
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

    if (main_radio_present)
    {
        tm.main_radio_present = 1;

        auto stats                    = getModule<RadioMain>()->getStats();
        tm.main_packet_tx_error_count = stats.send_errors;
        tm.main_tx_bitrate = main_tx_bitrate.update(stats.bits_tx_count);
        tm.main_packet_rx_success_count = stats.packet_rx_success_count;
        tm.main_packet_rx_drop_count    = stats.packet_rx_drop_count;
        tm.main_rx_bitrate = main_rx_bitrate.update(stats.bits_rx_count);
        tm.main_rx_rssi    = stats.rx_rssi;

        last_main_stats = stats;

        Logger::getInstance().log(MainRadioLog{
            tm.timestamp, tm.main_packet_tx_error_count, tm.main_tx_bitrate,
            tm.main_packet_rx_success_count, tm.main_packet_rx_drop_count,
            tm.main_rx_bitrate, tm.main_rx_rssi});
    }

    if (ethernet_present)
    {
        auto stats = getModule<EthernetGS>()->getState();

        tm.ethernet_present = 1;
        tm.ethernet_status  = (stats.link_up ? 1 : 0) |
                             (stats.full_duplex ? 2 : 0) |
                             (stats.based_100mbps ? 4 : 0);
    }

    mavlink_message_t msg;
    mavlink_msg_arp_tm_encode(Groundstation::ARP_SYSTEM_ID,
                              Groundstation::ARP_COMPONENT_ID, &msg, &tm);

    getModule<HubBase>()->dispatchIncomingMsg(msg);
}

void BoardStatus::GSRoutine()
{
    using namespace Groundstation;

    miosix::Thread::sleep(RADIO_STATUS_PERIOD);

    mavlink_arp_tm_t tm = {0};
    tm.timestamp        = TimestampTimer::getTimestamp();
    tm.battery_voltage  = -420.0;

    if (main_radio_present)
    {
        tm.main_radio_present = 1;

        auto stats                    = getModule<RadioMain>()->getStats();
        tm.main_packet_tx_error_count = stats.send_errors;
        tm.main_tx_bitrate = main_tx_bitrate.update(stats.bits_tx_count);
        tm.main_packet_rx_success_count = stats.packet_rx_success_count;
        tm.main_packet_rx_drop_count    = stats.packet_rx_drop_count;
        tm.main_rx_bitrate = main_rx_bitrate.update(stats.bits_rx_count);
        tm.main_rx_rssi    = stats.rx_rssi;

        last_main_stats = stats;
    }

    if (payload_radio_present)
    {
        tm.payload_radio_present = 1;

        auto stats = getModule<RadioPayload>()->getStats();
        tm.payload_packet_tx_error_count = stats.send_errors;
        tm.payload_tx_bitrate = payload_tx_bitrate.update(stats.bits_tx_count);
        tm.payload_packet_rx_success_count = stats.packet_rx_success_count;
        tm.payload_packet_rx_drop_count    = stats.packet_rx_drop_count;
        tm.payload_rx_bitrate = payload_rx_bitrate.update(stats.bits_rx_count);
        tm.payload_rx_rssi    = stats.rx_rssi;

        last_payload_stats = stats;
    }

    if (ethernet_present)
    {
        auto ethernet = getModule<EthernetGS>();
        auto stats    = ethernet->getState();

        tm.ethernet_present = 1;
        tm.ethernet_status  = (stats.link_up ? 1 : 0) |
                             (stats.full_duplex ? 2 : 0) |
                             (stats.based_100mbps ? 4 : 0);
    }

    mavlink_message_t msg;
    mavlink_msg_arp_tm_encode(Groundstation::GS_SYSTEM_ID, GS_COMPONENT_ID,
                              &msg, &tm);

    getModule<HubBase>()->dispatchIncomingMsg(msg);
}

void BoardStatus::run()
{
    while (!shouldStop())
    {
        if (isArp)
            arpRoutine();
        else
            GSRoutine();
    }
}