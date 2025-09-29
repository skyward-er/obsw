/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Federico Lolli, Nicolò Caruso
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

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Common/Config/GeneralConfig.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Ports/SerialLyraGS.h>
#include <Groundstation/LyraGS/Radio/Radio.h>
#include <algorithms/NAS/NASState.h>
#include <common/Events.h>
#include <common/MavlinkOrion.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <logger/Logger.h>
#include <sensors/SensorData.h>

#include <iostream>

using namespace Antennas;
using namespace Common;
using namespace Boardcore;
using namespace Groundstation;
using namespace miosix;

void Hub::dispatchOutgoingMsg(const mavlink_message_t& msg)
{
    logData(false, msg.msgid);

    TRACE("[info] Hub: Packet arrived from outgoing messages!!!\n");
    LyraGS::BoardStatus* status  = getModule<LyraGS::BoardStatus>();
    LyraGS::RadioMain* radioMain = getModule<LyraGS::RadioMain>();

    if (status->isMainRadioPresent() && msg.sysid == MAV_SYSID_MAIN)
    {
        if (!radioMain->sendMsg(msg))
            sendNack(msg, 306);
    }

    if (status->isPayloadRadioPresent() && msg.sysid == MAV_SYSID_PAYLOAD)
    {
        LyraGS::RadioPayload* radioPayload = getModule<LyraGS::RadioPayload>();
        if (!radioPayload->sendMsg(msg))
            sendNack(msg, 306);
    }

    // Message for ARP
    if (msg.sysid == MAV_SYSID_ARP)
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_ARP_COMMAND_TC:
            {
                // Create the map between the commands and the corresponding
                // events
                static const std::map<MavCommandList, Events> commandToEvent{
                    {MAV_CMD_FORCE_INIT, TMTC_ARP_FORCE_INIT},
                    {MAV_CMD_RESET_ALGORITHM, TMTC_ARP_RESET_ALGORITHM},
                    {MAV_CMD_ARP_FORCE_NO_FEEDBACK, TMTC_ARP_FORCE_NO_FEEDBACK},
                    {MAV_CMD_ARM, TMTC_ARP_ARM},
                    {MAV_CMD_DISARM, TMTC_ARP_DISARM},
                    {MAV_CMD_ARP_FOLLOW, TMTC_ARP_FOLLOW},
                    {MAV_CMD_CALIBRATE, TMTC_ARP_CALIBRATE},
                    {MAV_CMD_ENTER_TEST_MODE, TMTC_ARP_ENTER_TEST_MODE},
                    {MAV_CMD_EXIT_TEST_MODE, TMTC_ARP_EXIT_TEST_MODE},
                };

                MavCommandList commandId = static_cast<MavCommandList>(
                    mavlink_msg_arp_command_tc_get_command_id(&msg));

                if (commandId == MAV_CMD_FORCE_REBOOT)
                    reboot();

                auto it = commandToEvent.find(commandId);

                if (it != commandToEvent.end())
                    EventBroker::getInstance().post(it->second, TOPIC_TMTC);
                else
                    return sendNack(msg, 301);

                // Acknowledge the message
                sendAck(msg);
                break;
            }
            case MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC:
            {
                StepperList stepperId = static_cast<StepperList>(
                    mavlink_msg_set_stepper_angle_tc_get_stepper_id(&msg));
                float angle = mavlink_msg_set_stepper_angle_tc_get_angle(&msg);

                // The stepper is moved of 'angle' degrees
                ActuationStatus moved =
                    getModule<SMA>()->moveStepperDeg(stepperId, angle);
                if (moved == ActuationStatus::OK)
                    sendAck(msg);
                else
                    sendNack(msg, 302);
                break;
            }
            case MAVLINK_MSG_ID_SET_STEPPER_STEPS_TC:
            {
                StepperList stepperId = static_cast<StepperList>(
                    mavlink_msg_set_stepper_steps_tc_get_stepper_id(&msg));
                int16_t steps =
                    mavlink_msg_set_stepper_steps_tc_get_steps(&msg);

                // The stepper is moved of 'steps' steps
                ActuationStatus moved =
                    getModule<SMA>()->moveStepperSteps(stepperId, steps);
                if (moved == ActuationStatus::OK)
                    sendAck(msg);
                else
                    sendNack(msg, 303);
                break;
            }
            case MAVLINK_MSG_ID_SET_STEPPER_MULTIPLIER_TC:
            {
                StepperList stepperId = static_cast<StepperList>(
                    mavlink_msg_set_stepper_multiplier_tc_get_stepper_id(&msg));
                float multiplier =
                    mavlink_msg_set_stepper_multiplier_tc_get_multiplier(&msg);

                getModule<SMA>()->setMultipliers(stepperId, multiplier);
                sendAck(msg);
                break;
            }
            case MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC:
            {
                float latitude =
                    mavlink_msg_set_rocket_coordinates_arp_tc_get_latitude(
                        &msg);
                float longitude =
                    mavlink_msg_set_rocket_coordinates_arp_tc_get_longitude(
                        &msg);
                float height =
                    mavlink_msg_set_rocket_coordinates_arp_tc_get_height(&msg);

                GPSData gpsData;
                gpsData.gpsTimestamp = TimestampTimer::getTimestamp();
                gpsData.latitude     = latitude;
                gpsData.longitude    = longitude;
                gpsData.height       = height;
                gpsData.fix          = 3;
                gpsData.satellites   = 42;

                getModule<SMA>()->setRocketNASOrigin(gpsData);
                sendAck(msg);
                break;
            }
            case MAVLINK_MSG_ID_SET_ANTENNA_COORDINATES_ARP_TC:
            {
                float latitude =
                    mavlink_msg_set_antenna_coordinates_arp_tc_get_latitude(
                        &msg);
                float longitude =
                    mavlink_msg_set_antenna_coordinates_arp_tc_get_longitude(
                        &msg);
                float height =
                    mavlink_msg_set_antenna_coordinates_arp_tc_get_height(&msg);

                GPSData gpsData;
                gpsData.gpsTimestamp = TimestampTimer::getTimestamp();
                gpsData.latitude     = latitude;
                gpsData.longitude    = longitude;
                gpsData.height       = height;
                gpsData.fix          = 3;
                gpsData.satellites   = 42;

                getModule<SMA>()->setAntennaCoordinates(gpsData);
                sendAck(msg);
                break;
            }
            case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
            {
                mavlink_message_t msg_response;
                SystemTMList tmId = static_cast<SystemTMList>(
                    mavlink_msg_system_tm_request_tc_get_tm_id(&msg));

                // If it is handled create the msg_response, otherwise return
                // NACK
                switch (tmId)
                {
                    case SystemTMList::MAV_LOGGER_ID:
                    {
                        mavlink_logger_tm_t tm;

                        LoggerStats stats = Logger::getInstance().getStats();

                        tm.timestamp          = TimestampTimer::getTimestamp();
                        tm.log_number         = stats.logNumber;
                        tm.too_large_samples  = stats.tooLargeSamples;
                        tm.dropped_samples    = stats.droppedSamples;
                        tm.queued_samples     = stats.queuedSamples;
                        tm.buffers_filled     = stats.buffersFilled;
                        tm.buffers_written    = stats.buffersWritten;
                        tm.writes_failed      = stats.writesFailed;
                        tm.last_write_error   = stats.lastWriteError;
                        tm.average_write_time = stats.averageWriteTime;
                        tm.max_write_time     = stats.maxWriteTime;

                        mavlink_msg_logger_tm_encode(
                            SysIDs::MAV_SYSID_ARP,
                            Groundstation::GS_COMPONENT_ID, &msg_response, &tm);

                        break;
                    }
                    default:
                    {
                        sendNack(msg, 304);
                        return;
                    }
                }

                // Dispatching the created response message and then returning
                // the ACK
                dispatchIncomingMsg(msg_response);
                sendAck(msg);
                return;
            }
            default:
            {
                sendNack(msg, 305);
                return;
            }
        }
    }
}

void Hub::dispatchIncomingMsg(const mavlink_message_t& msg)
{
    LyraGS::SerialLyraGS* serial = getModule<LyraGS::SerialLyraGS>();
#if !defined(NO_MAVLINK_ON_SERIAL)
    serial->sendMsg(msg);
#else
    (void)serial;
#endif

    logData(true, msg.msgid);

    // Extracting NAS rocket state
    if (msg.msgid == MAVLINK_MSG_ID_ROCKET_FLIGHT_TM)
    {
        mavlink_rocket_flight_tm_t rocketTM;
        mavlink_msg_rocket_flight_tm_decode(&msg, &rocketTM);
        uint64_t timestamp = rocketTM.timestamp;
        TRACE(
            "[info][Radio/Sniffing] Hub: A FLIGHT_ROCKET_TM packet was "
            "received "
            "packet with ts %llu\n",
            timestamp);
        {
            Lock<FastMutex> lock(lastTMMutex);

            /* Messages older and within the discard interval are treated as old
             * messages*/
            if (timestamp > lastFlightTMTimestamp - DISCARD_MSG_DELAY &&
                timestamp <= lastFlightTMTimestamp)
                return;
            TRACE(
                "[info][Radio/Sniffing] Hub: A FLIGHT_ROCKET_TM packet is "
                "valid "
                "with ts %llu\n",
                timestamp);
            lastFlightTMTimestamp = timestamp;
        }
        NASState nasState{
            mavlink_msg_rocket_flight_tm_get_timestamp(&msg),
            Eigen::Matrix<float, 13, 1>(
                rocketTM.nas_n, rocketTM.nas_e, rocketTM.nas_d, rocketTM.nas_vn,
                rocketTM.nas_ve, rocketTM.nas_vd, rocketTM.nas_qx,
                rocketTM.nas_qy, rocketTM.nas_qz, rocketTM.nas_qw,
                rocketTM.nas_bias_x, rocketTM.nas_bias_y, rocketTM.nas_bias_z)};

        // Set the rocket NAS
        setRocketNasState(nasState);
    }
    else if (msg.msgid == MAVLINK_MSG_ID_ROCKET_STATS_TM)
    {
        mavlink_rocket_stats_tm_t rocketST;
        mavlink_msg_rocket_stats_tm_decode(&msg, &rocketST);
        TRACE(
            "[info][Radio/Sniffing] Hub: A ROCKET_STAT_TM packet was received "
            "packet with ts %llu\n",
            rocketST.timestamp);
        {
            Lock<FastMutex> lock(lastTMMutex);
            /* Messages older and within the discard interval are treated as old
             * messages*/
            if (rocketST.timestamp > lastStatsTMTimestamp - DISCARD_MSG_DELAY &&
                rocketST.timestamp <= lastStatsTMTimestamp)
                return;
            TRACE(
                "[info][Radio/Sniffing] Hub: A ROCKET_STAT_TM packet is valid, "
                "with ts %llu\n",
                rocketST.timestamp);
            lastStatsTMTimestamp = rocketST.timestamp;
        }

        // TODO: The origin should have its own struct since only timestamp and
        // [lat, lon, alt] are needed
        GPSData gpsState;
        getRocketOrigin(gpsState);
        gpsState.gpsTimestamp = rocketST.timestamp;
        gpsState.latitude     = rocketST.ref_lat;
        gpsState.longitude    = rocketST.ref_lon;
        gpsState.height       = rocketST.ref_alt;
        gpsState.fix          = 3;

        setRocketOrigin(gpsState);
    }

    // TODO: In case of sniffing the message should not be sent again
    LyraGS::EthernetGS* ethernet = getModule<LyraGS::EthernetGS>();
    ethernet->sendMsg(msg);
}

void Hub::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(SysIDs::MAV_SYSID_ARP, GS_COMPONENT_ID, &ackMsg,
                            msg.msgid, msg.seq);
    dispatchIncomingMsg(ackMsg);
}

bool Hub::getRocketOrigin(Boardcore::GPSData& rocketOrigin)
{
    Lock<FastMutex> lock(coordinatesMutex);
    rocketOrigin = lastRocketCoordinates;
    return originReceived;
}

bool Hub::getLastRocketNasState(Boardcore::NASState& nasState)
{
    Lock<FastMutex> lock(nasStateMutex);
    nasState     = lastRocketNasState;
    hasNewNasSet = false;
    return rocketNasSet;
}

bool Hub::hasNewNasState()
{
    Lock<FastMutex> lock(nasStateMutex);
    return hasNewNasSet;
}

void Hub::setRocketNasState(const NASState& newRocketNasState)
{
    Lock<FastMutex> lock(nasStateMutex);
    hasNewNasSet       = true;
    rocketNasSet       = true;
    lastRocketNasState = newRocketNasState;
    Logger::getInstance().log(newRocketNasState);
}

void Hub::setRocketOrigin(const GPSData& newRocketCoordinates)
{
    Lock<FastMutex> lock(coordinatesMutex);
    lastRocketCoordinates = newRocketCoordinates;
    originReceived        = true;
    Logger::getInstance().log(newRocketCoordinates);
}

void Hub::logData(bool isRocketRx, uint8_t messageId)
{
    Lock<FastMutex> lock(hubDataMutex);
    logHubData.timestamp = TimestampTimer::getTimestamp();
    logHubData.messageId = messageId;
    if (isRocketRx)
        logHubData.rocketRx = logHubData.rocketRx + 1;
    else
        logHubData.groundRx = logHubData.groundRx + 1;
    logHubData.cpuMean = CpuMeter::getCpuStats().mean;
    Logger::getInstance().log(logHubData);
}
