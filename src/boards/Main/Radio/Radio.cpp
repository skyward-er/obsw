/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Federico Mandelli
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
#include <Main/Buses.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/Xbee/APIFramesLog.h>
#include <radio/Xbee/ATCommands.h>

#include <memory>

using namespace Boardcore;
using namespace Common;

void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();

    if (modules.get<Main::Radio>()->transceiver != nullptr)
        modules.get<Main::Radio>()->transceiver->handleATTNInterrupt();
}

namespace Main
{

Radio::Radio(TaskScheduler* sched) : scheduler(sched) {}

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Create the SPI bus configuration
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create the xbee object
    transceiver = new Xbee::Xbee(
        modules.get<Buses>()->spi2, config, miosix::xbee::cs::getPin(),
        miosix::xbee::attn::getPin(), miosix::xbee::reset::getPin());
    transceiver->setOnFrameReceivedListener(
        std::bind(&Radio::onXbeeFrameReceived, this, std::placeholders::_1));

    Xbee::setDataRate(*transceiver, RadioConfig::XBEE_80KBPS_DATA_RATE,
                      RadioConfig::XBEE_TIMEOUT);

    enableExternalInterrupt(miosix::xbee::attn::getPin().getPort(),
                            miosix::xbee::attn::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    // Add periodic telemetry send task
    uint8_t result =
        scheduler->addTask([=]() { this->sendPeriodicMessage(); },
                           RadioConfig::RADIO_PERIODIC_TELEMETRY_PERIOD,
                           TaskScheduler::Policy::RECOVER);

    // Periodic stats

    // result *= scheduler->addTask(
    //     [&]()
    //     {
    //         this->enqueueMsg(
    //             modules.get<TMRepository>()->packSystemTm(MAV_STATS_ID, 0,
    //             0));
    //     },
    //     RadioConfig::RADIO_STATS_TELEMETRY_PERIOD,
    //     TaskScheduler::Policy::RECOVER);

    // Config mavDriver
    mavDriver = new MavDriver(
        transceiver,
        [=](MavDriver*, const mavlink_message_t& msg)
        { this->handleMavlinkMessage(msg); },
        RadioConfig::RADIO_SLEEP_AFTER_SEND,
        RadioConfig::RADIO_OUT_BUFFER_MAX_AGE);

    // Start the mavlink driver thread
    return mavDriver->start() && result != 0;
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(RadioConfig::MAV_SYSTEM_ID,
                            RadioConfig::MAV_COMP_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueueMsg(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(RadioConfig::MAV_SYSTEM_ID,
                             RadioConfig::MAV_COMP_ID, &nackMsg, msg.msgid,
                             msg.seq);
    enqueueMsg(nackMsg);
}

void Radio::logStatus() { Logger::getInstance().log(mavDriver->getStatus()); }

bool Radio::isStarted()
{
    return mavDriver->isStarted() && scheduler->isRunning();
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

void Radio::handleMavlinkMessage(const mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();

    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
        {
            // Do nothing, just add the ack to the queue
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            // Let the handle command reply to the message
            return handleCommand(msg);
        }
        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            SystemTMList tmId = static_cast<SystemTMList>(
                mavlink_msg_system_tm_request_tc_get_tm_id(&msg));

            if (tmId == SystemTMList::MAV_SENSORS_STATE_ID)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;
                auto sensorsState = ModuleManager::getInstance()
                                        .get<Sensors>()
                                        ->getSensorInfo();
                for (SensorInfo i : sensorsState)
                {
                    strcpy(tm.sensor_name, i.id.c_str());
                    tm.state = 0;
                    if (i.isEnabled)
                    {
                        tm.state += 1;
                    }
                    if (i.isInitialized)
                    {
                        tm.state += 2;
                    }
                    mavlink_msg_sensor_state_tm_encode(
                        RadioConfig::MAV_SYSTEM_ID, RadioConfig::MAV_COMP_ID,
                        &msg, &tm);
                    enqueueMsg(msg);
                }
            }
            else
            {

                // Add to the queue the respose
                mavlink_message_t response =
                    modules.get<TMRepository>()->packSystemTm(tmId, msg.msgid,
                                                              msg.seq);

                // Add the response to the queue
                enqueueMsg(response);

                // Check if the TM repo answered with a NACK. If so the function
                // must return to avoid sending a default ack
                if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
                {
                    return;
                }
            }
            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            SensorsTMList tmId = static_cast<SensorsTMList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg));

            // Add to the queue the respose
            mavlink_message_t response =
                modules.get<TMRepository>()->packSensorsTm(tmId, msg.msgid,
                                                           msg.seq);

            // Add the response to the queue
            enqueueMsg(response);

            // Check if the TM repo answered with a NACK. If so the function
            // must return to avoid sending a default ack
            if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
            {
                return;
            }

            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);
            if (modules.get<FlightModeManager>()->getStatus().state !=
                FlightModeManagerState::TEST_MODE)
            {
                return sendNack(msg);
            }
            modules.get<NASController>()->setReferenceAltitude(altitude);
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            temperature += 273.15;

            if (modules.get<FlightModeManager>()->getStatus().state !=
                FlightModeManagerState::TEST_MODE)
            {
                return sendNack(msg);
            }
            modules.get<NASController>()->setReferenceTemperature(temperature);
            break;
        }
        case MAVLINK_MSG_ID_SET_ORIENTATION_TC:
        {
            float yaw   = mavlink_msg_set_orientation_tc_get_yaw(&msg);
            float pitch = mavlink_msg_set_orientation_tc_get_pitch(&msg);
            float roll  = mavlink_msg_set_orientation_tc_get_roll(&msg);
            if (modules.get<FlightModeManager>()->getStatus().state !=
                FlightModeManagerState::TEST_MODE)
            {
                return sendNack(msg);
            }
            modules.get<NASController>()->setOrientation(yaw, pitch, roll);
            break;
        }
        case MAVLINK_MSG_ID_SET_COORDINATES_TC:
        {
            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);
            if (modules.get<FlightModeManager>()->getStatus().state !=
                FlightModeManagerState::TEST_MODE)
            {
                return sendNack(msg);
            }
            modules.get<NASController>()->setCoordinates(
                Eigen::Vector2f(latitude, longitude));
            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            // Send the event only if the flight mode manager is in test
            // mode
            if (modules.get<FlightModeManager>()->getStatus().state !=
                FlightModeManagerState::TEST_MODE)
            {
                return sendNack(msg);
            }

            EventBroker::getInstance().post(topicId, eventId);
            break;
        }
        default:
        {
            return sendNack(msg);
        }
    }

    // At the end send the ack message
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
        {MAV_CMD_FORCE_DEPLOYMENT, TMTC_FORCE_DEPLOYMENT},
        {MAV_CMD_START_LOGGING, TMTC_START_LOGGING},
        {MAV_CMD_STOP_LOGGING, TMTC_STOP_LOGGING},
        {MAV_CMD_FORCE_REBOOT, TMTC_RESET_BOARD},
        {MAV_CMD_ENTER_TEST_MODE, TMTC_ENTER_TEST_MODE},
        {MAV_CMD_EXIT_TEST_MODE, TMTC_EXIT_TEST_MODE},
        {MAV_CMD_START_RECORDING, TMTC_START_RECORDING},
        {MAV_CMD_STOP_RECORDING, TMTC_STOP_RECORDING},
    };
    switch (commandId)
    {
        case MAV_CMD_SAVE_CALIBRATION:
        {
            // Save the sensor calibration and adopt it
            ModuleManager::getInstance().get<Sensors>()->writeMagCalibration();
            break;
        }
        default:
        {
            // If the command is not a particular one, look for it inside
            // the map
            auto it = commandToEvent.find(commandId);

            if (it != commandToEvent.end())
            {
                EventBroker::getInstance().post(it->second, TOPIC_TMTC);
            }
            else
            {
                return sendNack(msg);
            }
        }
    }
    // Acknowledge the message
    sendAck(msg);
}

void Radio::sendPeriodicMessage()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Send all the queue messages
    {
        Lock<FastMutex> lock(queueMutex);

        for (uint8_t i = 0; i < messageQueueIndex; i++)
        {
            mavDriver->enqueueMsg(messageQueue[i]);
        }

        // Reset the index
        messageQueueIndex = 0;
    }

    mavDriver->enqueueMsg(
        modules.get<TMRepository>()->packSystemTm(MAV_FLIGHT_ID, 0, 0));
}

void Radio::enqueueMsg(const mavlink_message_t& msg)
{
    {
        Lock<FastMutex> lock(queueMutex);

        // Insert the message inside the queue only if there is enough space
        if (messageQueueIndex < RadioConfig::MAVLINK_QUEUE_SIZE)
        {
            messageQueue[messageQueueIndex] = msg;
            messageQueueIndex++;
        }
    }
}
}  // namespace Main
