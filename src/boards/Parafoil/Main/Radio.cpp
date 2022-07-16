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

#include <Parafoil/Configs/RadioConfig.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Configs/XbeeConfig.h>
#include <Parafoil/Main/Radio.h>
#include <Parafoil/ParafoilTest.h>
#include <Parafoil/TelemetriesTelecommands/TMRepository.h>
#include <Parafoil/Wing/WingTargetPositionData.h>
#include <common/events/Topics.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <radio/Xbee/ATCommands.h>
#include <utils/AeroUtils/AeroUtils.h>

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
    using namespace Parafoil;

    /*if (ParafoilTest::getInstance().radio->xbee != nullptr)
    {
        ParafoilTest::getInstance().radio->xbee->handleATTNInterrupt();
    }*/
    if (ParafoilTest::getInstance().radio->module != nullptr)
    {
        ParafoilTest::getInstance().radio->module->handleDioIRQ();
    }
}

namespace Parafoil
{

Radio::Radio(SPIBusInterface& xbee_bus, TaskScheduler* scheduler)
    : xbee_bus(xbee_bus)
{
    // Create a SPI configuration object
    SPIBusConfig config{};

    // Set the internal scheduler
    this->scheduler = scheduler;

    // Set the internal telemetry status to low rate
    this->HRtelemetry = false;

    // Add a clock divider config
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Instantiate the xbee object
    /*xbee = new Xbee::Xbee(xbee_bus, config,
                          XBEE_CS,
                          XBEE_ATTN,
                          XBEE_RESET);*/
    module = new SX1278(xbee_bus, XBEE_CS);

    // Create the mavlink driver
    mav_driver =
        new MavDriver(module, bind(&Radio::handleMavlinkMessage, this, _1, _2),
                      SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);
}

Radio::~Radio()
{
    // TODO destruct
}

void Radio::handleMavlinkMessage(MavDriver* driver,
                                 const mavlink_message_t& msg)
{
    // log status
    ParafoilTest::getInstance().radio->logStatus();

    LOG_DEBUG(logger, "Message received!");

    // acknowledge
    sendAck(msg);

    // handle TC
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_COMMAND_TC:  // basic command
        {
            LOG_DEBUG(logger, "Received NOARG command");
            uint8_t commandId = mavlink_msg_command_tc_get_command_id(&msg);

            // search for the corresponding event and post it
            auto it = tcMap.find(commandId);
            if (it != tcMap.end())
                EventBroker::getInstance().post(Event{it->second}, TOPIC_TMTC);
            else
                LOG_WARN(logger, "Unknown NOARG command {:d}", commandId);

            switch (commandId)
            {
                case MAV_CMD_FORCE_REBOOT:
                    SDlogger->stop();
                    LOG_INFO(logger, "Received command BOARD_RESET");
                    miosix::reboot();
                    break;
                case MAV_CMD_CLOSE_LOG:
                    SDlogger->stop();
                    sendSystemTelemetry(MAV_LOGGER_ID);
                    LOG_INFO(logger, "Received command CLOSE_LOG");
                    break;
                case MAV_CMD_START_LOGGING:
                    ParafoilTest::getInstance().SDlogger->start();
                    sendSystemTelemetry(MAV_LOGGER_ID);
                    LOG_INFO(logger, "Received command START_LOG");
                    break;
                case MAV_CMD_TEST_MODE:
                    // Use the test mode to apply the sequence
                    ParafoilTest::getInstance().wingController->start();
                    break;
                case MAV_CMD_FORCE_LANDING:
                    // Reset the servo position
                    ParafoilTest::getInstance().wingController->reset();
                    break;
                default:
                    break;
            }
            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:  // tm request
        {
            uint8_t tmId = mavlink_msg_system_tm_request_tc_get_tm_id(&msg);
            LOG_DEBUG(logger, "Received TM request : id = {:d}", tmId);

            // send corresponding telemetry or NACK
            sendSystemTelemetry(tmId);

            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            uint8_t tmId = mavlink_msg_sensor_tm_request_tc_get_sensor_id(&msg);

            // Send corresponding telemetry or NACK
            sendSensorTelemetry(tmId);

            break;
        }
        case MAVLINK_MSG_ID_SET_ALGORITHM_TC:
        {
            uint8_t algorithmId =
                mavlink_msg_set_algorithm_tc_get_algorithm_number(&msg);

            // Set the algorithm (invalid cases are checked inside the method)
            ParafoilTest::getInstance().wingController->selectAlgorithm(
                algorithmId);

            break;
        }
        case MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC:
        {
            float lat =
                mavlink_msg_set_target_coordinates_tc_get_latitude(&msg);
            float lon =
                mavlink_msg_set_target_coordinates_tc_get_longitude(&msg);

            // Set also the barometer reference
            ParafoilTest::getInstance().sensors->calibrate();

            // Set the target referencing to GPS coordinates
            UBXGPSData gps =
                ParafoilTest::getInstance().sensors->getGPSLastSample();
            if (gps.fix != 0)
            {
                auto targetPosition = Boardcore::Aeroutils::geodetic2NED(
                    Eigen::Vector2f(lat, lon),
                    Eigen::Vector2f(gps.latitude, gps.longitude));

                ParafoilTest::getInstance().wingController->setTargetPosition(
                    targetPosition);

                Logger::getInstance().log(WingTargetPositionData{
                    gps.latitude, gps.longitude, targetPosition(0),
                    targetPosition(1)});

                // Set also the initial position of the NASController
                ParafoilTest::getInstance().algorithms->nas->setInitialPosition(
                    Eigen::Vector2f(gps.latitude, gps.longitude));
            }
            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:  // post a raw event
        {
            LOG_DEBUG(logger, "Received RAW_EVENT command");

            // post given event on given topic
            EventBroker::getInstance().post(
                {mavlink_msg_raw_event_tc_get_event_id(&msg)},
                mavlink_msg_raw_event_tc_get_topic_id(&msg));
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

    // TODO REMOVE THIS ACK DONE BECAUSE OF ACK LOSS
    sendAck(msg);
}

bool Radio::sendSystemTelemetry(const uint8_t tm_id)
{
    // Enqueue the message
    bool result =
        mav_driver->enqueueMsg(TMRepository::getInstance().packSystemTM(tm_id));
    // TODO log the operation
    return result;
}

bool Radio::sendSensorTelemetry(const uint8_t tm_id)
{
    // Enqueue the message
    bool result =
        mav_driver->enqueueMsg(TMRepository::getInstance().packSensorTM(tm_id));
    return result;
}

void Radio::sendHRTelemetry()
{
    // sendTelemetry(MAV_HR_TM_ID);
    // TEST ONLY
    sendSystemTelemetry(MAV_FLIGHT_ID);
}

void Radio::sendLRTelemetry()
{
    // I send this telemetry if and only if the status is
    // in low rate telemetry
    sendSystemTelemetry(MAV_STATS_ID);
}

void Radio::sendSDLogTelemetry() { sendSystemTelemetry(MAV_LOGGER_ID); }

void Radio::sendServoTelemetry()
{
    mavlink_message_t msg;
    mavlink_servo_tm_t tm;

    tm.servo_id = PARAFOIL_SERVO1;
    tm.servo_position =
        ParafoilTest::getInstance().wingController->getServoPosition(
            PARAFOIL_SERVO1);
    mavlink_msg_servo_tm_encode(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &msg, &tm);
    mav_driver->enqueueMsg(msg);

    tm.servo_id = PARAFOIL_SERVO2;
    tm.servo_position =
        ParafoilTest::getInstance().wingController->getServoPosition(
            PARAFOIL_SERVO2);
    mavlink_msg_servo_tm_encode(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &msg, &tm);
    mav_driver->enqueueMsg(msg);
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    // Pack the ack message based on the received message
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid,
                            msg.seq);

    // Put the message in the queue
    mav_driver->enqueueMsg(ackMsg);
    // TODO log the thing
}

void Radio::toggleHRTelemetry()
{
    // Lock the kernel to avoid context switch during this toggle
    miosix::PauseKernelLock kLock;
    TaskScheduler::function_t HRfunction([=]() { sendHRTelemetry(); });
    HRtelemetry = !HRtelemetry;

    if (HRtelemetry)
    {
        // If we switched to hr telemetry on flight i change the period
        scheduler->removeTask(RADIO_HR_ID);
        // Add the same task with a faster period
        scheduler->addTask(HRfunction, HR_FLIGHT_UPDATE_PERIOD, RADIO_HR_ID);
    }
    else
    {
        // If we switched to hr telemetry on ground i change the period
        scheduler->removeTask(RADIO_HR_ID);
        // Add the same task with a slower period
        scheduler->addTask(HRfunction, HR_GROUND_UPDATE_PERIOD, RADIO_HR_ID);
    }
}

bool Radio::start()
{
    // Init the radio module
    init();

    // Start the mavlink driver
    bool result = mav_driver->start();

    // Start the scheduler
    // result &= scheduler -> start();

    return result;
}

void Radio::logStatus()
{
    // TODO log
}

void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
{
    // Log the thing
}

void Radio::init()
{
    // Create the lambdas to be called
    TaskScheduler::function_t HRfunction([=]() { sendHRTelemetry(); });
    TaskScheduler::function_t LRfunction([=]() { sendLRTelemetry(); });
    TaskScheduler::function_t SDfunction([=]() { sendSDLogTelemetry(); });

    // Enable external interrupt on F10 pin
    // TODO for xbee is FALLING
    enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::RISING_EDGE);

    // Init the radio module with default configs
    SX1278::Config conf;
    SX1278::Error initError = module->init(conf);

    // Check if there was an error initalizing the SX1278
    if (initError != SX1278::Error::NONE)
    {
        LOG_ERR(logger, "Error starting the SX1278 radio");
        return;
    }

    // Register the LR and HR tasks in the scheduler
    scheduler->addTask(HRfunction, 250, RADIO_HR_ID);
    // scheduler->addTask(LRfunction, LR_UPDATE_PERIOD, RADIO_LR_ID);
    scheduler->addTask(SDfunction, SD_UPDATE_PERIOD, SD_UPDATE_ID);
    scheduler->addTask([=]() { sendServoTelemetry(); }, 1000);

    // Set the frame receive callback
    // xbee -> setOnFrameReceivedListener(
    // bind(&Radio::onXbeeFrameReceived, this, _1));

    // Set the data rate
    // Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);
}

}  // namespace Parafoil
