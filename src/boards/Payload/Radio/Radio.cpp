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
#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/Buses.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Radio/RadioData.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>

#include <memory>

using namespace Boardcore;
using namespace Common;

#define SX1278_IRQ_DIO0 EXTI3_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI5_IRQHandlerImpl

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Payload::Radio>()->transceiver)
    {
        modules.get<Payload::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Payload::Radio>()->transceiver)
    {
        modules.get<Payload::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Payload::Radio>()->transceiver)
    {
        modules.get<Payload::Radio>()->transceiver->handleDioIRQ();
    }
}
namespace Payload
{

Radio::Radio(TaskScheduler& sched) : scheduler(sched) {}

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();
    // Config the transceiver
    SX1278Fsk::Config config = {
        .freq_rf    = 869000000,
        .freq_dev   = 50000,
        .bitrate    = 48000,
        .rx_bw      = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
        .afc_bw     = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000,
        .ocp        = 120,
        .power      = 13,
        .shaping    = Boardcore::SX1278Fsk::Config::Shaping::GAUSSIAN_BT_1_0,
        .dc_free    = Boardcore::SX1278Fsk::Config::DcFree::WHITENING,
        .enable_crc = false};

    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();

    transceiver = new SX1278Fsk(
        modules.get<Buses>()->spi6, miosix::radio::cs::getPin(),
        miosix::radio::dio0::getPin(), miosix::radio::dio1::getPin(),
        miosix::radio::dio3::getPin(), SPI::ClockDivider::DIV_128,
        std::move(frontend));

    // Config the radio
    SX1278Fsk::Error error = transceiver->init(config);

    // Add periodic telemetry send task
    uint8_t result =
        scheduler.addTask([&]() { this->sendPeriodicMessage(); },
                          RadioConfig::RADIO_PERIODIC_TELEMETRY_PERIOD,
                          TaskScheduler::Policy::RECOVER);
    result *= scheduler.addTask(
        [&]()
        {
            this->enqueueMsg(
                modules.get<TMRepository>()->packSystemTm(MAV_STATS_ID, 0, 0));
        },
        RadioConfig::RADIO_PERIODIC_TELEMETRY_PERIOD * 2,
        TaskScheduler::Policy::RECOVER);

    // Config mavDriver
    mavDriver = new MavDriver(
        transceiver,
        [=](MavDriver*, const mavlink_message_t& msg)
        { this->handleMavlinkMessage(msg); },
        RadioConfig::RADIO_SLEEP_AFTER_SEND,
        RadioConfig::RADIO_OUT_BUFFER_MAX_AGE);

    // Check radio failure
    if (error != SX1278Fsk::Error::NONE)
    {
        return false;
    }

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
    return mavDriver->isStarted() && scheduler.isRunning();
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
        case MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_servo_tm_request_tc_get_servo_id(&msg));

            // Add to the queue the respose
            mavlink_message_t response =
                modules.get<TMRepository>()->packServoTm(servoId, msg.msgid,
                                                         msg.seq);

            // Add the response to the queue
            mavDriver->enqueueMsg(response);

            // Check if the TM repo answered with a NACK. If so the function
            // must return to avoid sending a default ack
            if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
            {
                return;
            }

            break;
        }
        case MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_set_servo_angle_tc_get_servo_id(&msg));
            float angle = mavlink_msg_set_servo_angle_tc_get_angle(&msg);

            // Move the servo, if it fails send a nack
            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode) ||
                !modules.get<Actuators>()->setServo(servoId, angle))
            {
                return sendNack(msg);
            }

            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            // Send nack if the FMM is not in test mode
            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode) ||
                !modules.get<Actuators>()->wiggleServo(servoId))
            {
                return sendNack(msg);
            }

            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_reset_servo_tc_get_servo_id(&msg));

            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode) ||
                !modules.get<Actuators>()->setServo(servoId, 0))
            {
                return sendNack(msg);
            }

            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);

            modules.get<NASController>()->setReferenceAltitude(altitude);

            RadioSetterParameter log{};
            log.timestamp   = TimestampTimer::getTimestamp();
            log.refAltitude = altitude;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            modules.get<NASController>()->setReferenceTemperature(temperature);

            RadioSetterParameter log{};
            log.timestamp      = TimestampTimer::getTimestamp();
            log.refTemperature = temperature;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

            modules.get<AltitudeTrigger>()->setDeploymentAltitude(altitude);

            RadioSetterParameter log{};
            log.timestamp   = TimestampTimer::getTimestamp();
            log.depAltitude = altitude;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_SET_ORIENTATION_TC:
        {
            float yaw   = mavlink_msg_set_orientation_tc_get_yaw(&msg);
            float pitch = mavlink_msg_set_orientation_tc_get_pitch(&msg);
            float roll  = mavlink_msg_set_orientation_tc_get_roll(&msg);

            modules.get<NASController>()->setOrientation(yaw, pitch, roll);

            RadioSetterParameter log{};
            log.timestamp = TimestampTimer::getTimestamp();
            log.yawOri    = yaw;
            log.pitchOri  = pitch;
            log.rollOri   = roll;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_SET_COORDINATES_TC:
        {
            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);

            modules.get<NASController>()->setCoordinates(
                Eigen::Vector2f(latitude, longitude));

            RadioSetterParameter log{};
            log.timestamp = TimestampTimer::getTimestamp();
            log.latCord   = latitude;
            log.lonCord   = longitude;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC:
        {
            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);

            modules.get<WingController>()->setTargetPosition(
                Eigen::Vector2f(latitude, longitude));

            RadioSetterParameter log{};
            log.timestamp     = TimestampTimer::getTimestamp();
            log.latTargetCord = latitude;
            log.lonTargetCord = longitude;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_SET_ALGORITHM_TC:
        {
            uint8_t algoID =
                mavlink_msg_set_algorithm_tc_get_algorithm_number(&msg);

            if (!modules.get<WingController>()->selectAlgorithm(algoID))
            {
                return sendNack(msg);
            }
            RadioSetterParameter log{};
            log.timestamp = TimestampTimer::getTimestamp();
            log.algoId    = algoID;
            Logger::getInstance().log(log);
            break;
        }
        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            // Send the event only if the flight mode manager is in test mode
            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode))
            {
                return sendNack(msg);
            }

            EventBroker::getInstance().post(topicId, eventId);
            break;
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
            if (!ModuleManager::getInstance()
                     .get<Sensors>()
                     ->writeMagCalibration())
            {
                return sendNack(msg);
            }
            break;
        }
        default:
        {
            // If the command is not a particular one, look for it inside the
            // map
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
}  // namespace Payload
