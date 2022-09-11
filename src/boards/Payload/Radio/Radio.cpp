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
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/TMRepository/TMRepository.h>
#include <Payload/Wing/AltitudeTrigger.h>
#include <Payload/Wing/WingController.h>
#include <common/events/Events.h>
#include <drivers/interrupt/external_interrupts.h>
#include <radio/Xbee/APIFramesLog.h>
#include <radio/Xbee/ATCommands.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace miosix;
using namespace Boardcore;
using namespace Payload::RadioConfig;
using namespace Common;

// XBee interrupt
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    if (Payload::Radio::getInstance().xbee != nullptr)
        Payload::Radio::getInstance().xbee->handleATTNInterrupt();
}

namespace Payload
{

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

Radio::Radio()
{
    // Create the SPI bus configuration
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create the xbee object
    xbee = new Xbee::Xbee(
        Buses::getInstance().spi2, config, miosix::xbee::cs::getPin(),
        miosix::xbee::attn::getPin(), miosix::xbee::reset::getPin());
    xbee->setOnFrameReceivedListener(
        bind(&Radio::onXbeeFrameReceived, this, _1));

    Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);

    mavDriver =
        new MavDriver(xbee, bind(&Radio::handleMavlinkMessage, this, _1, _2), 0,
                      MAV_OUT_BUFFER_MAX_AGE);

    enableExternalInterrupt(miosix::xbee::attn::getPin().getPort(),
                            miosix::xbee::attn::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    // Add to the scheduler the periodic telemetries
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            mavDriver->enqueueMsg(
                TMRepository::getInstance().packSystemTm(MAV_FLIGHT_ID, 0, 0));
        },
        FLIGHT_TM_PERIOD, FLIGHT_TM_TASK_ID);
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            mavDriver->enqueueMsg(
                TMRepository::getInstance().packSystemTm(MAV_STATS_ID, 0, 0));
        },
        STATS_TM_PERIOD, STATS_TM_TASK_ID);
}

void Radio::onXbeeFrameReceived(Boardcore::Xbee::APIFrame& frame)
{
    using namespace Xbee;
    bool logged = false;
    switch (frame.frameType)
    {
        case FTYPE_AT_COMMAND:
        {
            ATCommandFrameLog dest;
            logged = ATCommandFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                Logger::getInstance().log(dest);
            }
            break;
        }
        case FTYPE_AT_COMMAND_RESPONSE:
        {
            ATCommandResponseFrameLog dest;
            logged = ATCommandResponseFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                Logger::getInstance().log(dest);
            }
            break;
        }
        case FTYPE_MODEM_STATUS:
        {
            ModemStatusFrameLog dest;
            logged = ModemStatusFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                Logger::getInstance().log(dest);
            }
            break;
        }
        case FTYPE_TX_REQUEST:
        {
            TXRequestFrameLog dest;
            logged = TXRequestFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                Logger::getInstance().log(dest);
            }
            break;
        }
        case FTYPE_TX_STATUS:
        {
            TXStatusFrameLog dest;
            logged = TXStatusFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                Logger::getInstance().log(dest);
            }
            break;
        }
        case FTYPE_RX_PACKET_FRAME:
        {
            RXPacketFrameLog dest;
            logged = RXPacketFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                Logger::getInstance().log(dest);
            }
            break;
        }
    }

    if (!logged)
    {
        APIFrameLog api;
        APIFrameLog::fromAPIFrame(frame, &api);
        Logger::getInstance().log(api);
    }
}

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

            // Send multiple packets for some telemetries
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

                        memset(tm.sensor_id, 0, sizeof(tm.sensor_id));
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
                    auto response = TMRepository::getInstance().packSystemTm(
                        tmId, msg.msgid, msg.seq);

                    mavDriver->enqueueMsg(response);

                    if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
                        return;
                    else
                        break;
                }
            }
            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            SensorsTMList sensorId = static_cast<SensorsTMList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_id(&msg));

            auto response = TMRepository::getInstance().packSensorsTm(
                sensorId, msg.msgid, msg.seq);

            mavDriver->enqueueMsg(response);

            if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
                return;
            else
                break;
        }
        case MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_servo_tm_request_tc_get_servo_id(&msg));

            auto response = TMRepository::getInstance().packServoTm(
                servoId, msg.msgid, msg.seq);

            mavDriver->enqueueMsg(response);

            if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
                return;
            else
                break;
        }
        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_set_servo_angle_tc_get_servo_id(&msg));
            float angle = mavlink_msg_set_servo_angle_tc_get_angle(&msg);

            // Move the servo, if it fails send a nack
            if (!(FlightModeManager::getInstance().getStatus().state ==
                      FlightModeManagerState::TEST_MODE &&
                  Actuators::getInstance().setServoAngle(servoId, angle)))
                return sendNack(msg);

            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            // Wiggle the servo, if it fails send a nack
            if (!(FlightModeManager::getInstance().getStatus().state ==
                      FlightModeManagerState::TEST_MODE &&
                  Actuators::getInstance().wiggleServo(servoId)))
                return sendNack(msg);

            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_reset_servo_tc_get_servo_id(&msg));

            // Reset the servo, if it fails send a nack
            if (!(FlightModeManager::getInstance().getStatus().state ==
                      FlightModeManagerState::TEST_MODE &&
                  Actuators::getInstance().setServo(servoId, 0)))
                return sendNack(msg);

            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);

            NASController::getInstance().setReferenceAltitude(altitude);
            Sensors::getInstance().pitotSetReferenceAltitude(altitude);
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            NASController::getInstance().setReferenceTemperature(temperature);
            Sensors::getInstance().pitotSetReferenceTemperature(temperature);
            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

            AltitudeTrigger::getInstance().setDeploymentAltitude(altitude);
            break;
        }
        case MAVLINK_MSG_ID_SET_ORIENTATION_TC:
        {
            float yaw   = mavlink_msg_set_orientation_tc_get_yaw(&msg);
            float pitch = mavlink_msg_set_orientation_tc_get_pitch(&msg);
            float roll  = mavlink_msg_set_orientation_tc_get_roll(&msg);

            NASController::getInstance().setOrientation(yaw, pitch, roll);
            break;
        }
        case MAVLINK_MSG_ID_SET_COORDINATES_TC:
        {
            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);

            NASController::getInstance().setCoordinates(
                Eigen::Vector2f(latitude, longitude));
            break;
        }
        case MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC:
        {
            float latitude =
                mavlink_msg_set_target_coordinates_tc_get_latitude(&msg);

            float longitude =
                mavlink_msg_set_target_coordinates_tc_get_longitude(&msg);

            WingController::getInstance().setTargetPosition(
                Eigen::Vector2f(latitude, longitude));

            break;
        }
        case MAVLINK_MSG_ID_SET_ALGORITHM_TC:
        {
            uint8_t algorithm =
                mavlink_msg_set_algorithm_tc_get_algorithm_number(&msg);

            WingController::getInstance().selectAlgorithm(algorithm);

            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            // Send the event only if the flight mode manager is in test mode
            if (FlightModeManager::getInstance().getStatus().state ==
                FlightModeManagerState::TEST_MODE)
                EventBroker::getInstance().post(topicId, eventId);
            else
                return sendNack(msg);

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

    static const std::map<MavCommandList, Events> commandToEvent{
        {MAV_CMD_ARM, TMTC_ARM},
        {MAV_CMD_DISARM, TMTC_DISARM},
        {MAV_CMD_CALIBRATE, TMTC_CALIBRATE},
        {MAV_CMD_FORCE_INIT, TMTC_FORCE_INIT},
        {MAV_CMD_FORCE_LAUNCH, TMTC_FORCE_LAUNCH},
        {MAV_CMD_FORCE_LANDING, TMTC_FORCE_LANDING},
        {MAV_CMD_FORCE_APOGEE, TMTC_FORCE_APOGEE},
        {MAV_CMD_FORCE_EXPULSION, TMTC_FORCE_EXPULSION},
        {MAV_CMD_FORCE_MAIN, TMTC_FORCE_MAIN},
        {MAV_CMD_START_LOGGING, TMTC_START_LOGGING},
        {MAV_CMD_STOP_LOGGING, TMTC_STOP_LOGGING},
        {MAV_CMD_FORCE_REBOOT, TMTC_RESET_BOARD},
        {MAV_CMD_ENTER_TEST_MODE, TMTC_ENTER_TEST_MODE},
        {MAV_CMD_EXIT_TEST_MODE, TMTC_EXIT_TEST_MODE},
        {MAV_CMD_START_RECORDING, TMTC_START_RECORDING},
        {MAV_CMD_STOP_RECORDING, TMTC_STOP_RECORDING},
    };
    auto it = commandToEvent.find(commandId);

    if (it != commandToEvent.end())
        EventBroker::getInstance().post(it->second, TOPIC_TMTC);
    else
        return sendNack(msg);

    // Acknowledge the message
    sendAck(msg);
}

}  // namespace Payload
