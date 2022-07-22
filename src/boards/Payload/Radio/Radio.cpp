/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Payload/Actuators/Actuators.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Sensors/Sensors.h>
#include <common/events/Events.h>
#include <drivers/interrupt/external_interrupts.h>
#include <radio/Xbee/ATCommands.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace miosix;
using namespace Boardcore;
using namespace Payload::RadioConfig;
using namespace Common;

/**
 * @brief We must define the interrupt handler. This calls
 * the message handler which is: handleMavlinkMessage
 */
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    if (Payload::Radio::getInstance().xbee != nullptr)
        Payload::Radio::getInstance().xbee->handleATTNInterrupt();
}

namespace Payload
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

            LOG_DEBUG(logger, "Received system telemetry request, id: {}",
                      tmId);

            // Send multiple packets for the TASK STATS telemetry
            switch (tmId)
            {
                case SystemTMList::MAV_PIN_OBS_ID:
                {
                    auto pinDataVector =
                        PinHandler::getInstance().getPinsData();

                    for (auto pinData : pinDataVector)
                    {
                        mavlink_message_t msgToSend;
                        mavlink_pin_tm_t tm;

                        tm.timestamp = TimestampTimer::getTimestamp();
                        tm.pin_id    = pinData.first;
                        tm.last_change_timestamp =
                            pinData.second.lastStateTimestamp;
                        tm.changes_counter = pinData.second.changesCount;
                        tm.current_state   = pinData.second.lastState;

                        mavlink_msg_pin_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                  RadioConfig::MAV_COMPONENT_ID,
                                                  &msgToSend, &tm);

                        mavDriver->enqueueMsg(msgToSend);
                    }

                    break;
                }
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
                    sendSystemTm(tmId, msg.msgid, msg.seq);
                    return;
                }
            }
            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            SensorsTMList tmId = static_cast<SensorsTMList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_id(&msg));

            LOG_DEBUG(logger, "Received system telemetry request, id: {}",
                      tmId);

            sendSensorsTm(tmId, msg.msgid, msg.seq);
            return;
        }
        case MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_id(&msg));

            LOG_DEBUG(logger, "Received servo telemetry request, id: {}",
                      servoId);

            sendServoTm(servoId, msg.msgid, msg.seq);
            return;
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
                return sendNack(msg);

            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            LOG_DEBUG(logger, "Received wiggle servo command, servoId: {}",
                      servoId);

            switch (servoId)
            {
                case AIRBRAKES_SERVO:
                    EventBroker::getInstance().post(TOPIC_ABK, ABK_WIGGLE);
                    break;
                case EXPULSION_SERVO:
                    EventBroker::getInstance().post(TOPIC_DPL, DPL_WIGGLE);
                    break;

                default:
                    sendNack(msg);
                    return;
            }

            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_reset_servo_tc_get_servo_id(&msg));

            LOG_DEBUG(logger, "Received reset servo command, servoId: {}",
                      servoId);

            switch (servoId)
            {
                case AIRBRAKES_SERVO:
                    EventBroker::getInstance().post(TOPIC_ABK, ABK_RESET);
                    break;
                case EXPULSION_SERVO:
                    EventBroker::getInstance().post(TOPIC_DPL, DPL_RESET);
                    break;

                default:
                    sendNack(msg);
                    return;
            }

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
        {
            LOG_DEBUG(logger, "Received command arm");

            EventBroker::getInstance().post(TMTC_ARM, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_DISARM:
        {
            LOG_DEBUG(logger, "Received command disarm");

            EventBroker::getInstance().post(TMTC_DISARM, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_FORCE_LAUNCH:
        {
            LOG_DEBUG(logger, "Received command force launch");

            EventBroker::getInstance().post(TMTC_FORCE_LAUNCH, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_FORCE_LANDING:
        {
            LOG_DEBUG(logger, "Received command force landing");

            EventBroker::getInstance().post(TMTC_FORCE_LANDING, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_FORCE_EXPULSION:
        {
            LOG_DEBUG(logger, "Received command force expulsion");

            EventBroker::getInstance().post(TMTC_FORCE_DROGUE, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_FORCE_MAIN:
        {
            LOG_DEBUG(logger, "Received command force main");

            EventBroker::getInstance().post(TMTC_FORCE_MAIN, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_START_LOGGING:
        {
            LOG_DEBUG(logger, "Received command start logging");
            Logger::getInstance().start();
            break;
        }
        case MAV_CMD_CLOSE_LOG:
        {
            LOG_DEBUG(logger, "Received command close log");

            Logger::getInstance().stop();

            break;
        }
        case MAV_CMD_FORCE_REBOOT:
        {
            reboot();

            break;
        }
        case MAV_CMD_TEST_MODE:
        {
            LOG_DEBUG(logger, "Received command test mode");

            EventBroker::getInstance().post(TMTC_ENTER_TEST_MODE, TOPIC_TMTC);

            break;
        }
        case MAV_CMD_START_RECORDING:
        {
            LOG_DEBUG(logger, "Received command start recording");

            // TODO: Apply command
            break;
        }
        case MAV_CMD_STOP_RECORDING:
        {
            LOG_DEBUG(logger, "Received command stop recording");

            // TODO: Apply command
            break;
        }

        default:
        {
            return sendNack(msg);
        }
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

bool Radio::sendSystemTm(const SystemTMList tmId, uint8_t msgId, uint8_t seq)
{
    // return mavDriver->enqueueMsg(
    //     TMRepository::getInstance().packSystemTm(tmId, msgId, seq));
    return false;
}

bool Radio::sendSensorsTm(const SensorsTMList sensorId, uint8_t msgId,
                          uint8_t seq)
{
    // return mavDriver->enqueueMsg(
    //     TMRepository::getInstance().packSensorsTm(sensorId, msgId, seq));
    return false;
}

bool Radio::sendServoTm(const ServosList servoId, uint8_t msgId, uint8_t seq)
{
    // return mavDriver->enqueueMsg(
    //     TMRepository::getInstance().packServoTm(servoId, msgId, seq));
    return false;
}

Radio::Radio()
{
    // Create the SPI bus configuration
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create the xbee object
    xbee = new Xbee::Xbee(
        Buses::getInstance().spi1, config, miosix::xbee::cs::getPin(),
        miosix::xbee::attn::getPin(), miosix::xbee::reset::getPin());

    mavDriver =
        new MavDriver(xbee, bind(&Radio::handleMavlinkMessage, this, _1, _2), 0,
                      MAV_OUT_BUFFER_MAX_AGE);

    // Add to the scheduler the flight and statistics telemetries
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]() { sendSystemTm(MAV_FLIGHT_ID, 0, 0); }, FLIGHT_TM_PERIOD,
        FLIGHT_TM_TASK_ID);
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]() { sendSystemTm(MAV_STATS_ID, 0, 0); }, STATS_TM_PERIOD,
        STATS_TM_TASK_ID);

    enableExternalInterrupt(miosix::xbee::attn::getPin().getPort(),
                            miosix::xbee::attn::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    // Set the callback
    xbee->setOnFrameReceivedListener(
        bind(&Radio::onXbeeFrameReceived, this, _1));

    // Set the data rate
    Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);
}

void Radio::onXbeeFrameReceived(Boardcore::Xbee::APIFrame& frame) {}

}  // namespace Payload