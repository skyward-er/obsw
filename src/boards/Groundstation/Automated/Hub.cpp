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

#include <Groundstation/Automated/Follower/Follower.h>
#include <Groundstation/Automated/Ports/Ethernet.h>
#include <Groundstation/Automated/Radio/Radio.h>
#include <Groundstation/Automated/Radio/RadioStatus.h>
#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <algorithms/NAS/NASState.h>
#include <logger/Logger.h>
#include <sensors/SensorData.h>

#include <iostream>

using namespace Antennas;
using namespace Boardcore;
using namespace Groundstation;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg)
{
    // TODO: Dispatch to correct radio using mavlink ids

    bool send_ok = false;

    RadioMain* radio = ModuleManager::getInstance().get<RadioMain>();
    send_ok |= radio->sendMsg(msg);

    // If both of the sends went wrong, just send a nack
    if (!send_ok)
    {
        sendNack(msg);
    }
}

void Hub::dispatchIncomingMsg(const mavlink_message_t& msg)
{
    Serial* serial = ModuleManager::getInstance().get<Serial>();
#if !defined(NO_MAVLINK_ON_SERIAL)
    serial->sendMsg(msg);
#else
    (void)serial;
#endif

    // Extracting NAS rocket state
    if (msg.msgid == MAVLINK_MSG_ID_ROCKET_FLIGHT_TM)
    {
        NASState nasState{
            mavlink_msg_rocket_flight_tm_get_timestamp(&msg),
            Eigen::Matrix<float, 13, 1>(
                mavlink_msg_rocket_flight_tm_get_nas_n(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_e(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_d(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_vn(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_ve(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_vd(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_qx(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_qy(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_qz(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_qw(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_bias_x(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_bias_y(&msg),
                mavlink_msg_rocket_flight_tm_get_nas_bias_z(&msg))};

        GPSData gpsState;
        gpsState.gpsTimestamp =
            mavlink_msg_rocket_flight_tm_get_timestamp(&msg);
        gpsState.latitude  = mavlink_msg_rocket_flight_tm_get_gps_lat(&msg);
        gpsState.longitude = mavlink_msg_rocket_flight_tm_get_gps_lon(&msg);
        gpsState.height    = mavlink_msg_rocket_flight_tm_get_gps_alt(&msg);
        gpsState.fix       = mavlink_msg_rocket_flight_tm_get_gps_fix(&msg);

        lastRocketNasState = nasState;
        lastRocketGpsState = gpsState;

        Logger::getInstance().log(nasState);
        Logger::getInstance().log(gpsState);
    }

    Ethernet* ethernet = ModuleManager::getInstance().get<Ethernet>();
    ethernet->sendMsg(msg);
}

/**
 * @param get rocket NAS state.
 */
NASState Hub::getLastRocketNasState() { return lastRocketNasState; }

GPSData Hub::getLastRocketGpsState() { return lastRocketGpsState; }
