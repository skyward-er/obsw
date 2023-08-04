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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Buses.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <drivers/interrupt/external_interrupts.h>
#include <radio/Xbee/APIFramesLog.h>
#include <radio/Xbee/ATCommands.h>

#include <functional>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace std;
using namespace placeholders;
using namespace miosix;
using namespace Boardcore;
using namespace Parafoil::RadioConfig;
using namespace Common;

// XBee interrupt
#ifndef USE_SERIAL_TRANSCEIVER
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();

    if (modules.get<Parafoil::Radio>()->transceiver != nullptr)
        modules.get<Parafoil::Radio>()->transceiver->handleATTNInterrupt();
}
#endif

namespace Parafoil
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

Radio::Radio() {}

bool Radio::isStarted() { return mavDriver->isStarted(); }

Boardcore::MavlinkStatus Radio::getMavlinkStatus()
{
    return mavDriver->getStatus();
}

void Radio::logStatus() { Logger::getInstance().log(mavDriver->getStatus()); }

bool Radio::startModule()
{
    ModuleManager& modules = ModuleManager::getInstance();

#if defined(USE_SERIAL_TRANSCEIVER)
    Boardcore::SerialTransceiver* transceiver;
    transceiver = new SerialTransceiver(modules.get<Buses>()->usart2);
#else
    // Create the SPI bus configuration
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;
    // Create the xbee object
    transceiver = new Xbee::Xbee(
        modules.get<Buses>()->spi2, config, miosix::xbee::cs::getPin(),
        miosix::xbee::attn::getPin(), miosix::xbee::reset::getPin());
    transceiver->setOnFrameReceivedListener(
        bind(&Radio::onXbeeFrameReceived, this, _1));

    Xbee::setDataRate(*transceiver, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);

    enableExternalInterrupt(miosix::xbee::attn::getPin().getPort(),
                            miosix::xbee::attn::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);
#endif

    mavDriver = new MavDriver(transceiver,
                              bind(&Radio::handleMavlinkMessage, this, _1, _2),
                              0, MAV_OUT_BUFFER_MAX_AGE);

    // Add to the scheduler the periodic telemetries
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            mavDriver->enqueueMsg(
                modules.get<TMRepository>()->packSystemTm(MAV_FLIGHT_ID, 0, 0));
        },
        FLIGHT_TM_PERIOD, FLIGHT_TM_TASK_ID);
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            mavDriver->enqueueMsg(
                modules.get<TMRepository>()->packSystemTm(MAV_STATS_ID, 0, 0));
        },
        STATS_TM_PERIOD, STATS_TM_TASK_ID);

    return mavDriver->start();
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
    ModuleManager& modules = ModuleManager::getInstance();

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
                    for (auto sensor :
                         modules.get<Sensors>()->getSensorsState())
                    {
                        mavlink_message_t msgToSend;
                        mavlink_sensor_state_tm_t tm;

                        memset(tm.sensor_name, 0, sizeof(tm.sensor_name));
                        sensor.first.copy(tm.sensor_name,
                                          sizeof(tm.sensor_name), 0);
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
                    auto response = modules.get<TMRepository>()->packSystemTm(
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
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg));

            auto response = modules.get<TMRepository>()->packSensorsTm(
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

            auto response = modules.get<TMRepository>()->packServoTm(
                servoId, msg.msgid, msg.seq);

            mavDriver->enqueueMsg(response);

            if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
                return;
            else
                break;
        }
        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);

            modules.get<NASController>()->setReferenceAltitude(altitude);
            modules.get<Sensors>()->pitotSetReferenceAltitude(altitude);
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            modules.get<NASController>()->setReferenceTemperature(temperature);
            modules.get<Sensors>()->pitotSetReferenceTemperature(temperature);
            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            /*float altitude =
                 mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

                //
               modules.get<AltitudeTrigger>()->setDeploymentAltitude(altitude);
                */
            break;
        }
        case MAVLINK_MSG_ID_SET_ORIENTATION_TC:
        {
            float yaw   = mavlink_msg_set_orientation_tc_get_yaw(&msg);
            float pitch = mavlink_msg_set_orientation_tc_get_pitch(&msg);
            float roll  = mavlink_msg_set_orientation_tc_get_roll(&msg);

            modules.get<NASController>()->setOrientation(yaw, pitch, roll);
            break;
        }
        case MAVLINK_MSG_ID_SET_COORDINATES_TC:
        {
            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);

            modules.get<NASController>()->setCoordinates(
                Eigen::Vector2f(latitude, longitude));
            break;
        }
        case MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC:
        {
            break;
        }
        case MAVLINK_MSG_ID_SET_ALGORITHM_TC:
        {

            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            // Send the event only if the flight mode manager is in test mode
            if (modules.get<FlightModeManager>()->getStatus().state ==
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

}  // namespace Parafoil