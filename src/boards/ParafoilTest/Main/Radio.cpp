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

#include <Main/Radio.h>
#include <TelemetriesTelecommands/TMRepository.h>
#include <Configs/XbeeConfig.h>
#include <Configs/RadioConfig.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/Xbee/ATCommands.h>
#include <drivers/interrupt/external_interrupts.h>
#include <ParafoilTest.h>
#include <common/events/Topics.h>

using std::bind;
using namespace std::placeholders;
using namespace Boardcore;

// Xbee ATTN interrupt
/**
 * @brief We must define the interrupt handler. This calls
 * the message handler which is: handleMavlinkMessage
 */
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    using namespace ParafoilTestDev;

    if (ParafoilTest::getInstance().radio->xbee != nullptr)
    {
        ParafoilTest::getInstance().radio->xbee->handleATTNInterrupt();
    }
}

namespace ParafoilTestDev
{
    /**
    * PUBLIC METHODS
    */
    Radio::Radio(SPIBusInterface& xbee_bus, TaskScheduler* scheduler) : xbee_bus(xbee_bus)
    {
        //Create a SPI configuration object
        SPIBusConfig config{};

        //Set the internal scheduler
        this -> scheduler = scheduler;

        //Set the internal telemetry status to low rate
        this -> HRtelemetry = false;

        //Add a clock divider config
        config.clockDivider = SPI::ClockDivider::DIV_16;

        //Instantiate the xbee object
        xbee = new Xbee::Xbee(xbee_bus, config,
                              XBEE_CS,
                              XBEE_ATTN,
                              XBEE_RESET);

        //Create the mavlink driver
        mav_driver = new MavDriver(xbee, 
                                   bind(&Radio::handleMavlinkMessage, this, _1, _2),
                                   SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);
        init();
    }

    Radio::~Radio()
    {
        //TODO destruct
    }

    void Radio::handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg)
    {
        // log status
        ParafoilTest::getInstance().radio->logStatus();

        LOG_DEBUG(logger, "Message received!");

        // acknowledge
        sendAck(msg);
        
        // handle TC
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_NOARG_TC:  // basic command
            {
                LOG_DEBUG(logger, "Received NOARG command");
                uint8_t commandId = mavlink_msg_noarg_tc_get_command_id(&msg);

                // search for the corresponding event and post it
                auto it = tcMap.find(commandId);
                if (it != tcMap.end())
                    sEventBroker.post(Event{it->second}, TOPIC_TMTC);
                else
                    LOG_WARN(logger, "Unknown NOARG command {:d}", commandId);

                switch (commandId)
                {
                    case MAV_CMD_BOARD_RESET:
                        SDlogger->stop();
                        LOG_INFO(logger, "Received command BOARD_RESET");
                        miosix::reboot();
                        break;
                    case MAV_CMD_CLOSE_LOG:
                        SDlogger->stop();
                        sendTelemetry(MAV_LOGGER_TM_ID);
                        LOG_INFO(logger, "Received command CLOSE_LOG");
                        break;
                    case MAV_CMD_START_LOGGING:
                        ParafoilTest::getInstance().startSDlogger();
                        sendTelemetry(MAV_LOGGER_TM_ID);
                        LOG_INFO(logger, "Received command START_LOG");
                        break;
                    default:
                        break;
                }
                break;
            }

            case MAVLINK_MSG_ID_TELEMETRY_REQUEST_TC:  // tm request
            {
                uint8_t tmId = mavlink_msg_telemetry_request_tc_get_board_id(&msg);
                LOG_DEBUG(logger, "Received TM request : id = {:d}", tmId);

                // send corresponding telemetry or NACK
                sendTelemetry(tmId);

                break;
            }
            /*
            case MAVLINK_MSG_ID_SET_INITIAL_ORIENTATION_TC:
            {
                float yaw = mavlink_msg_set_initial_orientation_tc_get_yaw(&msg);
                float pitch =
                    mavlink_msg_set_initial_orientation_tc_get_pitch(&msg);
                float roll = mavlink_msg_set_initial_orientation_tc_get_roll(&msg);
                LOG_DEBUG(logger,
                        "Received SET_INITIAL_ORIENTATION command. roll: {:f}, "
                        "pitch: {:f}, yaw: {:f}",
                        roll, pitch, yaw);
                ParafoilTest::getInstance().state_machines->setInitialOrientation(
                    roll, pitch, yaw);
                break;
            }

            case MAVLINK_MSG_ID_SET_INITIAL_COORDINATES_TC:
            {
                float latitude =
                    mavlink_msg_set_initial_coordinates_tc_get_latitude(&msg);
                float longitude =
                    mavlink_msg_set_initial_coordinates_tc_get_longitude(&msg);
                LOG_DEBUG(
                    logger,
                    "Received SET_INITIAL_COORDINATES command. latitude: {:f}, "
                    "longitude: {:f}",
                    latitude, longitude);
                ParafoilTest::getInstance().state_machines->setInitialCoordinates(
                    latitude, longitude);
                break;
            }*/
            case MAVLINK_MSG_ID_RAW_EVENT_TC:  // post a raw event
            {
                LOG_DEBUG(logger, "Received RAW_EVENT command");

                // post given event on given topic
                sEventBroker.post({mavlink_msg_raw_event_tc_get_Event_id(&msg)},
                                mavlink_msg_raw_event_tc_get_Topic_id(&msg));
                break;
            }

            case MAVLINK_MSG_ID_PING_TC:
            {
                LOG_DEBUG(logger, "Ping received");
                break;
            }

            default:
            {
                LOG_DEBUG(logger, "Received message is not of a known type");
                break;
            }
        }
    }

    bool Radio::sendTelemetry(const uint8_t tm_id)
    {
        //Enqueue the message
        bool result = mav_driver -> enqueueMsg(TMRepository::getInstance().packTM(tm_id));
        //TODO log the operation
        return result;
    }

    void Radio::sendHRTelemetry()
    {
        //I send this telemetry if and only if the status is
        //in high rate telemetry
        if(HRtelemetry)
        {
            sendTelemetry(MAV_HR_TM_ID);
        }
    }

    void Radio::sendLRTelemetry()
    {
        //I send this telemetry if and only if the status is
        //in low rate telemetry
        if(!HRtelemetry)
        {
            sendTelemetry(MAV_LR_TM_ID);
        }
    }

    void Radio::sendAck(const mavlink_message_t& msg)
    {
        mavlink_message_t ackMsg;
        //Pack the ack message based on the received message
        mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid, msg.seq);

        //Put the message in the queue
        mav_driver -> enqueueMsg(ackMsg);
        //TODO log the thing
    }

    void Radio::toggleHRTelemetry()
    {
        //Lock the kernel to avoid context switch during this toggle
        miosix::PauseKernelLock kLock;
        HRtelemetry = !HRtelemetry;
    }

    bool Radio::start()
    {
        //Start the mavlink driver
        bool result = mav_driver -> start();

        //Start the scheduler
        result &= scheduler -> start();

        return result;
    }

    void Radio::logStatus()
    {
        //TODO log
    }

    /**
     * PRIVATE METHODS
     */
    void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
    {
        //Log the thing
    }

    void Radio::init()
    {
        //Create the lambdas to be called
        TaskScheduler::function_t HRfunction([=]() {sendHRTelemetry();});
        TaskScheduler::function_t LRfunction([=]() {sendLRTelemetry();});
        
        //Register the LR and HR tasks in the scheduler
        //scheduler -> addTask(HRfunction, HR_UPDATE_PERIOD, RADIO_ID);
        //scheduler -> addTask(LRfunction, LR_UPDATE_PERIOD, RADIO_ID);

        //Set the frame receive callback
        xbee -> setOnFrameReceivedListener(
                bind(&Radio::onXbeeFrameReceived, this, _1));

        //Set the data rate
        Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);

        //Enable external interrupt on F10 pin
        enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::FALLING_EDGE);
    }
}