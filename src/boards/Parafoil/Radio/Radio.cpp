/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "Radio.h"

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Buses.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/TMRepository/TMRepository.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/Xbee/ATCommands.h>

#include <functional>

using namespace std;
using namespace miosix;
using namespace placeholders;
using namespace Boardcore;
using namespace Parafoil::RadioConfig;

void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    using namespace Parafoil;

    if (Radio::getInstance().transceiver != nullptr)
        Radio::getInstance().transceiver->handleDioIRQ();
}

namespace Parafoil
{

void Radio::handleMavlinkMessage(MavDriver* driver,
                                 const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
        {
            // Just send the ack at the end
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            return handleCommand(msg);
        }
        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            SystemTMList tmId = static_cast<SystemTMList>(
                mavlink_msg_system_tm_request_tc_get_tm_id(&msg));

            // Send multiple packets for the TASK STATS telemetry
            switch (tmId)
            {
                case SystemTMList::MAV_TASK_STATS_ID:
                {
                    auto statsVector = BoardScheduler::getInstance()
                                           .getScheduler()
                                           .getTaskStats();
                    uint64_t timestamp = TimestampTimer::getTimestamp();

                    for (auto stats : statsVector)
                    {
                        mavlink_message_t msgToSend;
                        mavlink_task_stats_tm_t tm;

                        tm.timestamp   = timestamp;
                        tm.task_id     = stats.id;
                        tm.task_period = stats.period;
                        tm.task_min    = stats.periodStats.minValue;
                        tm.task_max    = stats.periodStats.maxValue;
                        tm.task_mean   = stats.periodStats.mean;
                        tm.task_stddev = stats.periodStats.stdDev;

                        mavlink_msg_task_stats_tm_encode(
                            RadioConfig::MAV_SYSTEM_ID,
                            RadioConfig::MAV_COMPONENT_ID, &msgToSend, &tm);

                        mavDriver->enqueueMsg(msgToSend);
                    }

                    break;
                }
                case SystemTMList::MAV_SENSORS_STATE_ID:
                {
                    for (auto sensor : Sensors::getInstance().getSensorsState())
                    {
                        mavlink_message_t msgToSend;
                        mavlink_sensor_state_tm_t tm;

                        sensor.first.copy(tm.sensor_id, sizeof(tm.sensor_id),
                                          0);
                        tm.state = sensor.second;

                        mavlink_msg_sensor_state_tm_encode(
                            RadioConfig::MAV_SYSTEM_ID,
                            RadioConfig::MAV_COMPONENT_ID, &msgToSend, &tm);

                        mavDriver->enqueueMsg(msgToSend);
                    }

                    break;
                }
                default:
                {
                    sendSystemTm(tmId);
                    break;
                }
            }

            LOG_DEBUG(logger, "Received system telemetry request, id: {}",
                      tmId);
            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            SystemTMList tmId = static_cast<SystemTMList>(
                mavlink_msg_system_tm_request_tc_get_tm_id(&msg));
            sendSystemTm(tmId);

            LOG_DEBUG(logger, "Received system telemetry request, id: {}",
                      tmId);
            break;
        }
        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_set_servo_angle_tc_get_servo_id(&msg));
            float angle = mavlink_msg_set_servo_angle_tc_get_angle(&msg);

            LOG_DEBUG(logger,
                      "Received set servo angle command, servoId: {} angle: {}",
                      servoId, angle);

            // Move the servo, if failed send a nack
            if (!Actuators::getInstance().setServoAngle(servoId, angle))
            {
                sendNack(msg);
                return;
            }

            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            LOG_DEBUG(logger, "Received wiggle servo command, servoId: {}",
                      servoId);

            if (!Actuators::getInstance().wiggleServo(servoId))
                sendNack(msg);

            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_reset_servo_tc_get_servo_id(&msg));

            LOG_DEBUG(logger, "Received reset servo command, servoId: {}",
                      servoId);

            if (!Actuators::getInstance().setServo(servoId, 0))
                sendNack(msg);

            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);

            LOG_DEBUG(logger,
                      "Received set reference altitude command, altitude: {}",
                      altitude);

            // TODO: Apply command
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            LOG_DEBUG(
                logger,
                "Received set reference temperature command, temperature: {}",
                temperature);

            // TODO: Apply command
            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

            LOG_DEBUG(logger,
                      "Received set deployment altitude command, altitude: {}",
                      altitude);

            // TODO: Apply command
            break;
        }
        case MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC:
        {
            float yaw = mavlink_msg_set_initial_orientation_tc_get_yaw(&msg);
            float pitch =
                mavlink_msg_set_initial_orientation_tc_get_pitch(&msg);
            float roll = mavlink_msg_set_initial_orientation_tc_get_roll(&msg);

            LOG_DEBUG(logger,
                      "Received set initial orientation command, yaw: {}, "
                      "pitch: {}, roll: {}",
                      yaw, pitch, roll);

            // TODO: Apply command
            break;
        }
        case MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC:
        {
            float latitude =
                mavlink_msg_set_initial_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_initial_coordinates_tc_get_longitude(&msg);

            LOG_DEBUG(logger,
                      "Received set initial coordinates command, latitude: {}, "
                      "longitude: {}",
                      latitude, longitude);

            // TODO: Apply command
            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            LOG_DEBUG(logger,
                      "Received raw event command, topicId: {}, eventId{}",
                      topicId, eventId);

            EventBroker::getInstance().post(topicId, eventId);
            break;
        }

        default:
        {
            LOG_DEBUG(logger, "Received message is not of a known type");

            sendNack(msg);
            return;
        }
    }

    // Acknowledge the message
    sendAck(msg);
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    MavCommandList commandId = static_cast<MavCommandList>(
        mavlink_msg_command_tc_get_command_id(&msg));

    switch (commandId)
    {
        case MAV_CMD_ARM:
            LOG_DEBUG(logger, "Received command arm");

            // TODO: Apply command
            break;
        case MAV_CMD_DISARM:
            LOG_DEBUG(logger, "Received command disarm");

            // TODO: Apply command
            break;
        case MAV_CMD_FORCE_LAUNCH:
            LOG_DEBUG(logger, "Received command force launch");

            // TODO: Apply command
            break;
        case MAV_CMD_FORCE_LANDING:
            LOG_DEBUG(logger, "Received command force landing");

            // TODO: Apply command
            break;
        case MAV_CMD_FORCE_EXPULSION:
            LOG_DEBUG(logger, "Received command force expulsion");

            // TODO: Apply command
            break;
        case MAV_CMD_FORCE_MAIN:
            LOG_DEBUG(logger, "Received command force main");

            // TODO: Apply command
            break;
        case MAV_CMD_START_LOGGING:
            LOG_DEBUG(logger, "Received command start logging");
            Logger::getInstance().start();
            break;
        case MAV_CMD_CLOSE_LOG:
            LOG_DEBUG(logger, "Received command close log");
            Logger::getInstance().stop();
            break;
        case MAV_CMD_FORCE_REBOOT:
            reboot();
            break;
        case MAV_CMD_TEST_MODE:
            LOG_DEBUG(logger, "Received command test mode");

            // TODO: Apply command
            break;
        case MAV_CMD_START_RECORDING:
            LOG_DEBUG(logger, "Received command start recording");

            // TODO: Apply command
            break;
        case MAV_CMD_STOP_RECORDING:
            LOG_DEBUG(logger, "Received command stop recording");

            // TODO: Apply command
            break;

        default:
            return sendNack(msg);
    }

    // Acknowledge the message
    sendAck(msg);
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    mavDriver->enqueueMsg(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    LOG_DEBUG(logger, "Sending NACK for message {}", msg.msgid);
    mavlink_msg_nack_tm_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    mavDriver->enqueueMsg(nackMsg);
}

bool Radio::start() { return mavDriver->start(); }

bool Radio::isStarted() { return mavDriver->isStarted(); }

Boardcore::MavlinkStatus Radio::getMavlinkStatus()
{
    return mavDriver->getStatus();
}

void Radio::logStatus()
{
    Logger::getInstance().log(mavDriver->getStatus());
    // TODO: Add transceiver status logging
}

bool Radio::sendSystemTm(const SystemTMList tmId)
{
    bool result =
        mavDriver->enqueueMsg(TMRepository::getInstance().packSystemTm(tmId));
    return result;
}

bool Radio::sendSensorsTm(const SensorsTMList tmId)
{
    bool result =
        mavDriver->enqueueMsg(TMRepository::getInstance().packSensorsTm(tmId));
    return result;
}

Radio::Radio()
{
    transceiver = new SX1278(Buses::getInstance().spi4, sx1278::cs::getPin());

    mavDriver = new MavDriver(transceiver,
                              bind(&Radio::handleMavlinkMessage, this, _1, _2),
                              0, MAV_OUT_BUFFER_MAX_AGE);

    // Add to the scheduler the flight and statistics telemetries
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]() { sendSystemTm(MAV_FLIGHT_ID); }, FLIGHT_TM_PERIOD,
        FLIGHT_TM_TASK_ID);
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]() { sendSystemTm(MAV_STATS_ID); }, STATS_TM_PERIOD,
        STATS_TM_TASK_ID);
}

}  // namespace Parafoil
