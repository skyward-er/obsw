/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor, Federico Lolli, Nicolò Caruso
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
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Radio/Radio.h>
#include <algorithms/NAS/NASState.h>
#include <common/Events.h>
#include <common/Mavlink.h>
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
    // TODO: Dispatch to correct radio using mavlink ids
    bool send_ok           = false;
    ModuleManager& modules = ModuleManager::getInstance();

    LyraGS::RadioMain* radio = modules.get<LyraGS::RadioMain>();

    if (msg.sysid == MAV_SYSID_ARP)
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_ARP_COMMAND_TC:
            {
                // Create the map between the commands and the corresponding
                // events
                static const std::map<MavArpCommandList, Events> commandToEvent{
                    {MAV_ARP_CMD_FORCE_INIT, TMTC_ARP_FORCE_INIT},
                    {MAV_ARP_CMD_RESET_ALGORITHM, TMTC_ARP_RESET_ALGORITHM},
                    {MAV_ARP_CMD_RESET_BOARD, TMTC_ARP_RESET_BOARD},
                    {MAV_ARP_CMD_FORCE_NO_FEEDBACK, TMTC_ARP_FORCE_NO_FEEDBACK},
                    {MAV_ARP_CMD_ARM, TMTC_ARP_ARM},
                    {MAV_ARP_CMD_DISARM, TMTC_ARP_DISARM},
                    {MAV_ARP_CMD_FOLLOW, TMTC_ARP_FOLLOW},
                    {MAV_ARP_CMD_CALIBRATE, TMTC_ARP_CALIBRATE},
                    {MAV_ARP_CMD_ENTER_TEST_MODE, TMTC_ARP_ENTER_TEST_MODE},
                    {MAV_ARP_CMD_EXIT_TEST_MODE, TMTC_ARP_EXIT_TEST_MODE},
                };

                MavArpCommandList commandId = static_cast<MavArpCommandList>(
                    mavlink_msg_arp_command_tc_get_command_id(&msg));

                auto it = commandToEvent.find(commandId);

                if (it != commandToEvent.end())
                {
                    EventBroker::getInstance().post(it->second, TOPIC_TMTC);
                }
                else
                {
                    return sendNack(msg, 301);
                }

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
                    modules.get<SMA>()->moveStepperDeg(stepperId, angle);
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
                    modules.get<SMA>()->moveStepperSteps(stepperId, steps);
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

                modules.get<SMA>()->setMultipliers(stepperId, multiplier);
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
                gpsData.latitude   = latitude;
                gpsData.longitude  = longitude;
                gpsData.height     = height;
                gpsData.fix        = 3;
                gpsData.satellites = 42;

                modules.get<SMA>()->setInitialRocketCoordinates(gpsData);
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
                gpsData.latitude   = latitude;
                gpsData.longitude  = longitude;
                gpsData.height     = height;
                gpsData.fix        = 3;
                gpsData.satellites = 42;

                modules.get<SMA>()->setAntennaCoordinates(gpsData);
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
    send_ok |= radio->sendMsg(msg);

    // If both of the sends went wrong, just send a nack
    if (!send_ok)
    {
        sendNack(msg, 306);
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

        // Set the rocket NAS
        setRocketNasState(nasState);
        setRocketCoordinates(gpsState);

        Logger::getInstance().log(nasState);
        Logger::getInstance().log(gpsState);
    }

    LyraGS::Ethernet* ethernet =
        ModuleManager::getInstance().get<LyraGS::Ethernet>();
    ethernet->sendMsg(msg);
}

void Hub::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(SysIDs::MAV_SYSID_ARP, GS_COMPONENT_ID, &ackMsg,
                            msg.msgid, msg.seq);
    dispatchIncomingMsg(ackMsg);
}

GPSData Hub::getRocketCoordinates()
{
    Lock<FastMutex> lock(coordinatesMutex);
    return lastRocketCoordinates;
}

NASState Hub::getRocketNasState()
{
    Lock<FastMutex> lock(nasStateMutex);
    return lastRocketNasState;
}

void Hub::setRocketNasState(const NASState& newRocketNasState)
{
    Lock<FastMutex> lock(nasStateMutex);
    lastRocketNasState = newRocketNasState;
}

void Hub::setRocketCoordinates(const GPSData& newRocketCoordinates)
{
    Lock<FastMutex> lock(coordinatesMutex);
    lastRocketCoordinates = newRocketCoordinates;
}
