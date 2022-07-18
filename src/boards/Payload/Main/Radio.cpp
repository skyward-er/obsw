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
#include <Payload/Main/Radio.h>
#include <Payload/Payload.h>
#include <drivers/interrupt/external_interrupts.h>
#include <radio/Xbee/ATCommands.h>

using namespace Boardcore;
using namespace std::placeholders;

/**
 * @brief We must define the interrupt handler. This calls
 * the message handler which is: handleMavlinkMessage
 */
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    if (Payload::Payload::getInstance().radio->xbee != nullptr)
    {
        Payload::Payload::getInstance().radio->xbee->handleATTNInterrupt();
    }
}

namespace Payload
{
Radio::Radio(SPIBusInterface& xbeeBus, TaskScheduler* scheduler)
    : xbeeBus(xbeeBus)
{
    // Create the SPI bus configuration
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Set the internal scheduler
    this->scheduler = scheduler;

    // Create the xbee object
    xbee = new Xbee::Xbee(xbeeBus, config, miosix::xbee::cs::getPin(),
                          miosix::xbee::attn::getPin(),
                          miosix::xbee::reset::getPin());

    // Create the mavlink driver with the radio
    mavDriver =
        new MavDriver(xbee, bind(&Radio::handleMavlinkMessage, this, _1, _2),
                      SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);
}

Radio::~Radio()
{
    // Destruct the mavlink driver and the radio
    delete mavDriver;
    delete xbee;
}

void Radio::handleMavlinkMessage(MavDriver* driver,
                                 const mavlink_message_t& msg)
{
    // Log the status
    logStatus();

    // Send the ack
    sendAck(msg);

    // Handle the message
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            LOG_DEBUG(logger, "Received NOARG command");

            // Get the command id
            uint8_t commandId = mavlink_msg_command_tc_get_command_id(&msg);

            // Search for the corresponding event and post it
            auto it = tcMap.find(commandId);

            // If not the end of the map i post the command
            if (it != tcMap.end())
                EventBroker::getInstance().post(Event{it->second}, TOPIC_TMTC);
            else
                LOG_WARN(logger, "Unlnown NOARG command {:d}", commandId);

            // In case of basic commands i execute them
            switch (commandId)
            {
                case MAV_CMD_FORCE_REBOOT:
                {
                    LOG_INFO(logger, "Received command FORCE_REBOOT");
                    // Close the log and reset the board
                    SDlogger->stop();
                    miosix::reboot();

                    break;
                }
                case MAV_CMD_START_LOGGING:
                {
                    LOG_INFO(logger, "Received command START_LOGGING");
                    // Start the logging
                    SDlogger->start();
                    // Send the corresponding telemetry
                    sendSystemTelemetry(MAV_LOGGER_ID);

                    break;
                }
                case MAV_CMD_CLOSE_LOG:
                {
                    LOG_INFO(logger, "Received command CLOSE_LOG");
                    // Close the log
                    SDlogger->stop();
                    // Send the correspongind telemetry
                    sendSystemTelemetry(MAV_LOGGER_ID);

                    break;
                }
                default:
                {
                    break;
                }
            }

            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TELEMETRY_REQUEST_TC:
        {
            // System telemetry request
            uint8_t tmId =
                mavlink_msg_system_telemetry_request_tc_get_tm_id(&msg);
            LOG_DEBUG(logger, "Received a system TM request : id = {:d}", tmId);

            // Send the requested telemetry
            sendSystemTelemetry(tmId);

            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TELEMETRY_REQUEST_TC:
        {
            // Sensor telemetry request
            uint8_t tmId =
                mavlink_msg_sensor_telemetry_request_tc_get_sensor_id(&msg);
            LOG_DEBUG(logger, "Received a sensor TM request: id = {:d}", tmId);

            // Send the requested telemetry
            sendSensorTelemetry(tmId);

            break;
        }
        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            // Set the servo angle
            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            // Wiggle the specified servo
            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            // Resets the specified servo
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            // Sets the reference altitude for the altimeter
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            // Sets the reference temperature for the altimeter
            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            // Sets the deployment altitude for the main parachute
            break;
        }
        case MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC:
        {
            // Sets the initial orientation for the navigation system
            break;
        }
        case MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC:
        {
            // Sets the initial coordinates for the launchpad
            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            // Posts a raw event
            LOG_DEBUG(logger, "Received RAW EVENT command");

            // Post the event
            EventBroker::getInstance().post(
                {mavlink_msg_raw_event_tc_get_event_id(&msg)},
                mavlink_msg_raw_event_tc_get_topic_id(&msg));

            break;
        }
        case MAVLINK_MSG_ID_PING_TC:
        {
            // Received a ping
            LOG_DEBUG(logger, "Ping received");
            break;
        }
        default:
        {
            LOG_DEBUG(logger, "Received message of a non known type");
            break;
        }
    }
}

bool Radio::sendSystemTelemetry(const uint8_t tmId)
{
    bool result =
        mavDriver->enqueueMsg(TMRepository::getInstance().packSystemTM(tmId));

    // TODO log the operation
    return result;
}
bool Radio::sendSensorTelemetry(const uint8_t tmId)
{
    bool result =
        mavDriver->enqueueMsg(TMRepository::getInstance().packSensorTM(tmId));

    // TODO log the operation
    return result;
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;

    // Pack the ack message based on the received one
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid,
                            msg.seq);

    // Send the message
    mavDriver->enqueueMsg(ackMsg);
}

bool Radio::start()
{
    // Init the radio module
    init();

    // Start the mavlink driver and the scheduler
    return mavDriver->start() && scheduler->start();
}

void Radio::logStatus()
{
    // TODO log the internal radio status
}

void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
{
    // TODO log the received frame
}

void Radio::init()
{
    // Enable the external interrupt
    enableExternalInterrupt(miosix::xbee::attn::getPin().getPort(),
                            miosix::xbee::attn::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    // Set the callback
    xbee->setOnFrameReceivedListener(
        bind(&Radio::onXbeeFrameReceived, this, _1));

    // Set the data rate
    Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);
}
}  // namespace Payload