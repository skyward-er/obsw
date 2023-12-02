/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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
#include <RIG/Actuators/Actuators.h>
#include <RIG/Buses.h>
#include <RIG/CanHandler/CanHandler.h>
#include <RIG/Radio/Radio.h>
#include <RIG/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIG/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace Boardcore;
using namespace miosix;

void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<RIG::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) EXTI12_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<RIG::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) EXTI13_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<RIG::Radio>()->transceiver->handleDioIRQ();
    }
}
namespace RIG
{

Radio::Radio()
{
    // Initialize previous state to avoid undefined behaviour
    previousState.arm_switch           = 0;
    previousState.filling_valve_btn    = 0;
    previousState.ignition_btn         = 0;
    previousState.quick_connector_btn  = 0;
    previousState.release_pressure_btn = 0;
    previousState.start_tars_btn       = 0;
    previousState.venting_valve_btn    = 0;
}

Radio::~Radio()
{
    mavDriver->stop();
    delete mavDriver;
}

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Config the transceiver
    SX1278Lora::Config config{};
    config.power            = 2;
    config.ocp              = 0;  // Over current protection
    config.coding_rate      = SX1278Lora::Config::Cr::CR_1;
    config.spreading_factor = SX1278Lora::Config::Sf::SF_7;

    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txEn::getPin(),
                                        radio::rxEn::getPin());

    transceiver = new SX1278Lora(
        modules.get<Buses>()->spi4, radio::cs::getPin(), radio::dio0::getPin(),
        radio::dio1::getPin(), radio::dio3::getPin(), SPI::ClockDivider::DIV_64,
        std::move(frontend));

    SX1278Lora::Error error = transceiver->init(config);

    // Config mavDriver
    mavDriver = new MavDriver(
        transceiver, Config::Radio::MAV_PING_MSG_ID,
        [=](MavDriver*, const mavlink_message_t& msg)
        { this->handleMavlinkMessage(msg); },
        Config::Radio::MAV_SLEEP_AFTER_SEND);

    // This thread allows the radio to exit a possible deadlock situation where
    // the driver waits for a missed interrupt. By default the priority is MAX -
    // 1
    radioBackupDIO = std::thread(
        [=]()
        {
            while (true)
            {
                Thread::sleep(
                    5000);  // wait 5 seconds and then launch the interrupt
                {
                    FastInterruptDisableLock lock;
                    transceiver->handleDioIRQ();
                }
            }
        });

    // In case of failure the mav driver must be created at least
    if (error != SX1278Lora::Error::NONE)
    {
        return false;
    }

    return mavDriver->start();
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    mavDriver->enqueueMsg(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    mavDriver->enqueueMsg(nackMsg);
}

void Radio::logStatus()
{
    MavlinkStatus stats = mavDriver->getStatus();
    SDlogger.log(stats);
}

bool Radio::isStarted() { return mavDriver->isStarted(); }

void Radio::handleMavlinkMessage(const mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
        {
            // Do nothing just add the ack to the queue
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            // Important to return instead of breaking because handle command
            // can decide whether to send ack or nack
            return handleCommand(msg);
        }
        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            SystemTMList tmId = static_cast<SystemTMList>(
                mavlink_msg_system_tm_request_tc_get_tm_id(&msg));
            switch (tmId)
            {
                default:
                {
                    mavlink_message_t response =
                        modules.get<TMRepository>()->packSystemTm(
                            tmId, msg.msgid, msg.seq);

                    mavDriver->enqueueMsg(response);

                    if (response.msgid == MAVLINK_MSG_ID_NACK_TM)
                    {
                        // No break to let tmRepo decide for nack
                        return;
                    }
                    break;
                }
            }
            break;
        }
        case MAVLINK_MSG_ID_CONRIG_STATE_TC:
        {
            sendAck(msg);

            // Send the RIG state messages
            mavDriver->enqueueMsg(modules.get<TMRepository>()->packSystemTm(
                SystemTMList::MAV_GSE_ID, msg.msgid, msg.seq));
            mavDriver->enqueueMsg(modules.get<TMRepository>()->packSystemTm(
                SystemTMList::MAV_MOTOR_ID, msg.msgid, msg.seq));

            mavlink_conrig_state_tc_t state;
            mavlink_msg_conrig_state_tc_decode(&msg, &state);

            // Extract the buttons data and if there is a slope post the event
            if (previousState.arm_switch == 0 && state.arm_switch == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                    Common::TOPIC_TARS);
                    EventBroker::getInstance().post(Common::TMTC_ARM,
                                                    Common::TOPIC_MOTOR);
                    modules.get<CanHandler>()->sendEvent(
                        Common::CanConfig::EventId::ARM);

                    lastManualCommand = getTick();
                }
            }
            if (previousState.filling_valve_btn == 0 &&
                state.filling_valve_btn == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                    Common::TOPIC_TARS);
                    modules.get<Actuators>()->toggleServo(
                        ServosList::FILLING_VALVE);

                    lastManualCommand = getTick();
                }
            }
            if (previousState.ignition_btn == 0 && state.ignition_btn == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                    Common::TOPIC_TARS);
                    EventBroker::getInstance().post(Common::MOTOR_IGNITION,
                                                    Common::TOPIC_MOTOR);
                    lastManualCommand = getTick();
                }
            }
            if (previousState.quick_connector_btn == 0 &&
                state.quick_connector_btn == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                    Common::TOPIC_TARS);
                    modules.get<Actuators>()->toggleServo(
                        ServosList::DISCONNECT_SERVO);
                    lastManualCommand = getTick();
                }
            }
            if (previousState.release_pressure_btn == 0 &&
                state.release_pressure_btn == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                    Common::TOPIC_TARS);
                    modules.get<Actuators>()->toggleServo(
                        ServosList::RELEASE_VALVE);

                    lastManualCommand = getTick();
                }
            }
            if (previousState.start_tars_btn == 0 && state.start_tars_btn == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_START_TARS,
                                                    Common::TOPIC_TARS);
                    lastManualCommand = getTick();
                }
            }
            if (previousState.venting_valve_btn == 0 &&
                state.venting_valve_btn == 1)
            {
                if (getTick() > lastManualCommand +
                                    Config::Radio::RADIO_LAST_COMMAND_THRESHOLD)
                {
                    EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                    Common::TOPIC_TARS);
                    modules.get<Actuators>()->toggleServo(
                        ServosList::VENTING_VALVE);

                    lastManualCommand = getTick();
                }
            }

            if (previousState.arm_switch == 1 && state.arm_switch == 0)
            {
                EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                                Common::TOPIC_TARS);
                EventBroker::getInstance().post(Common::TMTC_DISARM,
                                                Common::TOPIC_MOTOR);
                modules.get<CanHandler>()->sendEvent(
                    Common::CanConfig::EventId::DISARM);

                lastManualCommand = getTick();
            }

            previousState = state;

            // Important to return and not to break to avoid a redundant ack
            return;
        }
        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            EventBroker::getInstance().post(Common::MOTOR_MANUAL_ACTION,
                                            Common::TOPIC_TARS);
            // Wiggle only the machine is in ready state
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            if (modules.get<GroundModeManager>()->getStatus().state ==
                GroundModeManagerState::READY)
            {
                bool result = modules.get<Actuators>()->wiggleServo(servo);

                if (!result)
                {
                    sendNack(msg);
                    return;
                }
            }
            else
            {
                sendNack(msg);
                return;
            }

            break;
        }
        case MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC:
        {
            uint32_t timing =
                mavlink_msg_set_atomic_valve_timing_tc_get_maximum_timing(&msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_atomic_valve_timing_tc_get_servo_id(&msg));
            modules.get<Actuators>()->setTiming(servo, timing);
            break;
        }
        case MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC:
        {
            float aperture =
                mavlink_msg_set_valve_maximum_aperture_tc_get_maximum_aperture(
                    &msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_valve_maximum_aperture_tc_get_servo_id(&msg));
            modules.get<Actuators>()->setMaximumAperture(servo, aperture);
            break;
        }
        case MAVLINK_MSG_ID_SET_IGNITION_TIME_TC:
        {
            uint32_t timing = mavlink_msg_set_ignition_time_tc_get_timing(&msg);
            modules.get<GroundModeManager>()->setIgnitionTime(timing);
            break;
        }
        default:
        {
            // Unknown message
            sendNack(msg);
            return;
        }
    }
    sendAck(msg);
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    uint8_t command = mavlink_msg_command_tc_get_command_id(&msg);
    switch (command)
    {
        case MAV_CMD_START_LOGGING:
        {
            bool result = Logger::getInstance().start();

            // If the log reached the maximum allowed number then send a nack
            if (!result || Logger::getInstance().getCurrentLogNumber() ==
                               Logger::getMaxFilenameNumber())
            {
                sendNack(msg);
                return;
            }

            break;
        }
        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            break;
        }
        case MAV_CMD_CALIBRATE:
        {
            EventBroker::getInstance().post(Common::TMTC_CALIBRATE,
                                            Common::TOPIC_MOTOR);
            ModuleManager::getInstance().get<CanHandler>()->sendEvent(
                Common::CanConfig::EventId::CALIBRATE);
            break;
        }
        default:
        {
            // Send a nack for unknown message
            sendNack(msg);
            return;
        }
    }

    sendAck(msg);
}

}  // namespace RIG