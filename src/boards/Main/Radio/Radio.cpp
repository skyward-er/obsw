/* Copyright (c) 2023 Skyward Experimental Rocketry
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
#include <Main/Actuators/Actuators.h>
#include <Main/AltitudeTrigger/AltitudeTrigger.h>
#include <Main/Buses.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>

#include <Eigen/Core>

using namespace Boardcore;
using namespace Common;
using namespace Eigen;

#define SX1278_IRQ_DIO0 EXTI3_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI5_IRQHandlerImpl

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Main::Radio>()->transceiver)
    {
        modules.get<Main::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Main::Radio>()->transceiver)
    {
        modules.get<Main::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Main::Radio>()->transceiver)
    {
        modules.get<Main::Radio>()->transceiver->handleDioIRQ();
    }
}
namespace Main
{

Radio::Radio(TaskScheduler* sched) : scheduler(sched) {}

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Config the transceiver
    SX1278Fsk::Config config;
    config.freq_rf    = 419000000;
    config.freq_dev   = 50000;
    config.bitrate    = 48000;
    config.rx_bw      = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000;
    config.afc_bw     = Boardcore::SX1278Fsk::Config::RxBw::HZ_125000;
    config.ocp        = 120;
    config.power      = 13;
    config.shaping    = Boardcore::SX1278Fsk::Config::Shaping::GAUSSIAN_BT_1_0;
    config.dc_free    = Boardcore::SX1278Fsk::Config::DcFree::WHITENING;
    config.enable_crc = false;

    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();

    transceiver = new SX1278Fsk(
        modules.get<Buses>()->spi6, miosix::radio::cs::getPin(),
        miosix::radio::dio0::getPin(), miosix::radio::dio1::getPin(),
        miosix::radio::dio3::getPin(), SPI::ClockDivider::DIV_64,
        std::move(frontend));

    // Config the radio
    SX1278Fsk::Error error = transceiver->init(config);

    // Add periodic telemetry send task
    uint8_t result =
        scheduler->addTask([&]() { this->sendPeriodicMessage(); },
                           RadioConfig::RADIO_PERIODIC_TELEMETRY_PERIOD,
                           TaskScheduler::Policy::RECOVER);
    result *= scheduler->addTask(
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

void Radio::logStatus() {}

bool Radio::isStarted()
{
    return mavDriver->isStarted() && scheduler->isRunning();
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

            break;
        }
        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            SensorsTMList sensorId = static_cast<SensorsTMList>(
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg));
            mavlink_message_t response =
                modules.get<TMRepository>()->packSensorsTm(sensorId, msg.msgid,
                                                           msg.seq);
            enqueueMsg(response);

            // If the response is a nack the method returns
            if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
            {
                return;
            }
            break;
        }
        case MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC:
        {
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_servo_tm_request_tc_get_servo_id(&msg));
            mavlink_message_t response =
                modules.get<TMRepository>()->packServoTm(servo, msg.msgid,
                                                         msg.seq);
            enqueueMsg(response);

            // If the response is a nack the method returns
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
            float position = mavlink_msg_set_servo_angle_tc_get_angle(&msg);

            // Send nack if the FMM is not in test mode
            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode))
            {
                return sendNack(msg);
            }

            // If the state is test mode, the servo is set to the correct angle
            modules.get<Actuators>()->setServoPosition(servoId, position);
            break;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            // Send nack if the FMM is not in test mode
            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode))
            {
                return sendNack(msg);
            }

            // If the state is test mode, the wiggle is done
            modules.get<Actuators>()->wiggleServo(servoId);

            break;
        }
        case MAVLINK_MSG_ID_RESET_SERVO_TC:
        {
            ServosList servoId = static_cast<ServosList>(
                mavlink_msg_reset_servo_tc_get_servo_id(&msg));

            // Send nack if the FMM is not in test mode
            if (!modules.get<FlightModeManager>()->testState(
                    &FlightModeManager::state_test_mode))
            {
                return sendNack(msg);
            }

            // Set the servo position to 0
            modules.get<Actuators>()->setServoPosition(servoId, 0);

            break;
        }
        case MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_deployment_altitude_tc_get_dpl_altitude(&msg);

            modules.get<AltitudeTrigger>()->setDeploymentAltitude(altitude);
            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC:
        {
            float altitude =
                mavlink_msg_set_reference_altitude_tc_get_ref_altitude(&msg);

            modules.get<ADAController>()->setReferenceAltitude(altitude);
            modules.get<NASController>()->setReferenceAltitude(altitude);

            break;
        }
        case MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC:
        {
            float temperature =
                mavlink_msg_set_reference_temperature_tc_get_ref_temp(&msg);

            modules.get<ADAController>()->setReferenceTemperature(temperature);
            modules.get<NASController>()->setReferenceTemperature(temperature);

            break;
        }
        case MAVLINK_MSG_ID_SET_COORDINATES_TC:
        {
            float latitude = mavlink_msg_set_coordinates_tc_get_latitude(&msg);
            float longitude =
                mavlink_msg_set_coordinates_tc_get_longitude(&msg);

            Vector2f coordinates{latitude, longitude};

            modules.get<NASController>()->setCoordinates(coordinates);
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
        default:
        {
            LOG_DEBUG(logger, "Received message is not of a known type");
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
    ModuleManager& modules = ModuleManager::getInstance();

    // Create the map between the commands and the corresponding events
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
            modules.get<Sensors>()->writeMagCalibration();
            break;
        }
        case MAV_CMD_START_LOGGING:
        {
            bool result = Logger::getInstance().start();

            // In case the logger is not started send to GS the result
            if (!result)
            {
                return sendNack(msg);
            }
            break;
        }
        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
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
        modules.get<TMRepository>()->packSystemTm(MAV_MOTOR_ID, 0, 0));
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