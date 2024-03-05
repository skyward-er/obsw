/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/Buses.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIGv2/StateMachines/TARS1/TARS1.h>
#include <common/Events.h>
#include <common/Radio.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

#include <atomic>

using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv2;

SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Lora* instance = gRadio;
    if (instance)
    {
        instance->handleDioIRQ();
    }
}

void setIRQRadio(SX1278Lora* radio)
{
    FastInterruptDisableLock dl;
    gRadio = radio;
}

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

bool Radio::isStarted() { return started; }

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txEn::getPin(),
                                        radio::rxEn::getPin());

    // Setup transceiver
    radio = std::make_unique<SX1278Lora>(
        modules.get<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(radio.get());

    // Initialize radio
    auto result = radio->init(RIG_RADIO_CONFIG);
    if (result != SX1278Lora::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize RIG radio");
        return false;
    }

    // Initialize mavdriver
    mavDriver = std::make_unique<MavDriver>(
        radio.get(),
        [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); },
        Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize RIG mav driver");
        return false;
    }

    started = true;
    return true;
}

void Radio::enqueuePacket(const mavlink_message_t& msg)
{
    queuedPackets.put(msg);
}

void Radio::flushPackets()
{
    // Flush all packets of the queue
    size_t count = queuedPackets.count();
    for (size_t i = 0; i < count; i++)
    {
        try
        {
            mavDriver->enqueueMsg(queuedPackets.pop());
        }
        catch (...)
        {
            // This shouldn't happen, but still try to prevent it
        }
    }
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueuePacket(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    enqueuePacket(nackMsg);
}

Boardcore::MavlinkStatus Radio::getMavStatus()
{
    if(mavDriver) {
        return mavDriver->getStatus();
    } else {
        return {};
    }
}

void Radio::handleMessage(const mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
        {
            sendAck(msg);
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_TC:
        {
            handleCommand(msg);
            break;
        }

        case MAVLINK_MSG_ID_CONRIG_STATE_TC:
        {
            handleConrigState(msg);
            break;
        }

        case MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC:
        {
            uint8_t tmId = mavlink_msg_system_tm_request_tc_get_tm_id(&msg);

            mavlink_message_t tm;
            if (tmId == MAV_SENSORS_STATE_ID)
            {
                sendAck(msg);

                auto sensors = modules.get<Sensors>()->getSensorInfos();
                for (auto sensor : sensors)
                {
                    mavlink_sensor_state_tm_t tm2;

                    strcpy(tm2.sensor_name, sensor.id.c_str());
                    tm2.state = (sensor.isInitialized ? 1 : 0) |
                                (sensor.isEnabled ? 2 : 0);

                    mavlink_msg_sensor_state_tm_encode(
                        Config::Radio::MAV_SYSTEM_ID,
                        Config::Radio::MAV_COMPONENT_ID, &tm, &tm2);
                    enqueuePacket(tm);
                }
            }
            else if (packSystemTm(tmId, tm))
            {
                sendAck(msg);
                enqueuePacket(tm);
            }
            else
            {
                sendNack(msg);
            }

            break;
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            if (modules.get<GroundModeManager>()->isDisarmed())
            {
                if (modules.get<Actuators>()->wiggleServo(servo))
                {
                    sendAck(msg);
                }
                else
                {
                    sendNack(msg);
                }
            }
            else
            {
                sendNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC:
        {
            uint32_t time =
                mavlink_msg_set_atomic_valve_timing_tc_get_maximum_timing(&msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_atomic_valve_timing_tc_get_servo_id(&msg));

            if (modules.get<Actuators>()->setOpeningTime(servo, time))
            {
                sendAck(msg);
            }
            else
            {
                sendNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC:
        {
            float aperture =
                mavlink_msg_set_valve_maximum_aperture_tc_get_maximum_aperture(
                    &msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_valve_maximum_aperture_tc_get_servo_id(&msg));

            if (modules.get<Actuators>()->setMaxAperture(servo, aperture))
            {
                sendAck(msg);
            }
            else
            {
                sendNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_IGNITION_TIME_TC:
        {
            uint32_t timing = mavlink_msg_set_ignition_time_tc_get_timing(&msg);
            modules.get<GroundModeManager>()->setIgnitionTime(timing);

            sendAck(msg);
            break;
        }

        default:
        {
            // Unrecognized packet
            sendNack(msg);
            break;
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    uint8_t cmd = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmd)
    {
        case MAV_CMD_START_LOGGING:
        {
            if (!Logger::getInstance().start())
            {
                sendNack(msg);
            }
            else
            {
                sendAck(msg);
            }
            break;
        }

        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            sendAck(msg);
            break;
        }

        case MAV_CMD_CALIBRATE:
        {
            EventBroker::getInstance().post(TMTC_CALIBRATE, TOPIC_MOTOR);
            sendAck(msg);
            break;
        }

        case MAV_CMD_FORCE_INIT:
        {
            EventBroker::getInstance().post(TMTC_FORCE_INIT, TOPIC_MOTOR);
            sendAck(msg);
            break;
        }

        case MAV_CMD_FORCE_REBOOT:
        {
            EventBroker::getInstance().post(TMTC_RESET_BOARD, TOPIC_MOTOR);
            sendAck(msg);
            break;
        }

        default:
        {
            // Unrecognized command
            sendNack(msg);
            break;
        }
    }
}

bool Radio::packSystemTm(uint8_t tmId, mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();
    switch (tmId)
    {
        case MAV_SYS_ID:
        {
            mavlink_sys_tm_t tm;

            tm.timestamp    = TimestampTimer::getTimestamp();
            tm.logger       = Logger::getInstance().isStarted() ? 1 : 0;
            tm.event_broker = EventBroker::getInstance().isRunning() ? 1 : 0;
            // What? Why is this here? Of course the radio is started!
            tm.radio           = isStarted() ? 1 : 0;
            tm.sensors         = modules.get<Sensors>()->isStarted() ? 1 : 0;
            tm.board_scheduler = 0;  // TODO(davide.mor): No BoardScheduler yet

            mavlink_msg_sys_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            return true;
        }

        case MAV_LOGGER_ID:
        {
            mavlink_logger_tm_t tm;

            LoggerStats stats = Logger::getInstance().getStats();

            tm.timestamp          = TimestampTimer::getTimestamp();
            tm.log_number         = stats.logNumber;
            tm.too_large_samples  = stats.tooLargeSamples;
            tm.dropped_samples    = stats.droppedSamples;
            tm.queued_samples     = stats.queuedSamples;
            tm.buffers_filled     = stats.buffersFilled;
            tm.buffers_written    = stats.buffersWritten;
            tm.writes_failed      = stats.writesFailed;
            tm.last_write_error   = stats.lastWriteError;
            tm.average_write_time = stats.averageWriteTime;
            tm.max_write_time     = stats.maxWriteTime;

            mavlink_msg_logger_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                         Config::Radio::MAV_COMPONENT_ID, &msg,
                                         &tm);
            return true;
        }

        case MAV_MAVLINK_STATS:
        {
            mavlink_mavlink_stats_tm_t tm;

            MavlinkStatus stats = modules.get<Radio>()->mavDriver->getStatus();

            tm.timestamp               = stats.timestamp;
            tm.n_send_queue            = stats.nSendQueue;
            tm.max_send_queue          = stats.maxSendQueue;
            tm.n_send_errors           = stats.nSendErrors;
            tm.msg_received            = stats.mavStats.msg_received;
            tm.buffer_overrun          = stats.mavStats.buffer_overrun;
            tm.parse_error             = stats.mavStats.parse_error;
            tm.parse_state             = stats.mavStats.parse_state;
            tm.packet_idx              = stats.mavStats.packet_idx;
            tm.current_rx_seq          = stats.mavStats.current_rx_seq;
            tm.current_tx_seq          = stats.mavStats.current_tx_seq;
            tm.packet_rx_success_count = stats.mavStats.packet_rx_success_count;
            tm.packet_rx_drop_count    = stats.mavStats.packet_rx_drop_count;

            mavlink_msg_mavlink_stats_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                                Config::Radio::MAV_COMPONENT_ID,
                                                &msg, &tm);
            return true;
        }

        case MAV_GSE_ID:
        {
            mavlink_gse_tm_t tm = {0};

            Sensors* sensors     = modules.get<Sensors>();
            Actuators* actuators = modules.get<Actuators>();

            tm.timestamp        = TimestampTimer::getTimestamp();
            tm.loadcell_rocket  = sensors->getTankWeight().load;
            tm.loadcell_vessel  = sensors->getVesselWeight().load;
            tm.filling_pressure = sensors->getFillingPress().pressure;
            tm.vessel_pressure  = sensors->getVesselPress().pressure;
            tm.filling_valve_state =
                actuators->isServoOpen(ServosList::FILLING_VALVE) ? 1 : 0;
            tm.venting_valve_state =
                actuators->isServoOpen(ServosList::VENTING_VALVE) ? 1 : 0;
            tm.release_valve_state =
                actuators->isServoOpen(ServosList::RELEASE_VALVE) ? 1 : 0;
            tm.main_valve_state =
                actuators->isServoOpen(ServosList::MAIN_VALVE) ? 1 : 0;
            tm.arming_state =
                modules.get<GroundModeManager>()->isArmed() ? 1 : 0;
            tm.ignition_state =
                modules.get<GroundModeManager>()->isIgniting() ? 1 : 0;
            tm.tars_state = modules.get<TARS1>()->isRefueling() ? 1 : 0;
            // TODO(davide.mor): Add the rest of these

            // Temporary hack to tell if the board initialized or not
            tm.main_board_status =
                modules.get<GroundModeManager>()->isDisarmed();

            tm.battery_voltage     = sensors->getBatteryVoltage().voltage;
            tm.current_consumption = sensors->getServoCurrent().current;

            mavlink_msg_gse_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            return true;
        }

        case MAV_MOTOR_ID:
        {
            mavlink_motor_tm_t tm = {0};

            Sensors* sensors = modules.get<Sensors>();

            tm.timestamp            = TimestampTimer::getTimestamp();
            tm.tank_temperature     = sensors->getTc1LastSample().temperature;
            tm.top_tank_pressure    = sensors->getTopTankPress().pressure;
            tm.bottom_tank_pressure = sensors->getBottomTankPress().pressure;
            tm.floating_level       = 69.0f;  // Lol
            // TODO(davide.mor): Add the rest of these

            tm.battery_voltage     = 0.0f;
            tm.current_consumption = sensors->getUmbilicalCurrent().current;

            mavlink_msg_motor_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                        Config::Radio::MAV_COMPONENT_ID, &msg,
                                        &tm);
            return true;
        }

        default:
        {
            return false;
        }
    }
}

void Radio::handleConrigState(const mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Acknowledge the state
    sendAck(msg);

    // Flush all pending packets
    flushPackets();

    // Send GSE and motor telemetry
    mavlink_message_t tm;
    packSystemTm(MAV_GSE_ID, tm);
    enqueuePacket(tm);
    packSystemTm(MAV_MOTOR_ID, tm);
    enqueuePacket(tm);

    mavlink_conrig_state_tc_t state;
    mavlink_msg_conrig_state_tc_decode(&msg, &state);

    long long currentTime = getTime();
    if (currentTime >
        lastManualActuation +
            (Config::Radio::LAST_COMMAND_THRESHOLD * Constants::NS_IN_MS))
    {
        // Ok we can accept new commands
        if (oldConrigState.arm_switch == 0 && state.arm_switch == 1)
        {
            // The ARM switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            EventBroker::getInstance().post(TMTC_ARM, TOPIC_MOTOR);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.ignition_btn == 0 && state.ignition_btn == 1)
        {
            // The ignition switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            EventBroker::getInstance().post(MOTOR_IGNITION, TOPIC_MOTOR);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.filling_valve_btn == 0 &&
            state.filling_valve_btn == 1)
        {
            // The filling switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            modules.get<Actuators>()->toggleServo(ServosList::FILLING_VALVE);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.quick_connector_btn == 0 &&
            state.quick_connector_btn == 1)
        {
            // The quick conector switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            modules.get<Actuators>()->toggleServo(ServosList::DISCONNECT_SERVO);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.release_pressure_btn == 0 &&
            state.release_pressure_btn == 1)
        {
            // The release switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            modules.get<Actuators>()->toggleServo(ServosList::RELEASE_VALVE);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.venting_valve_btn == 0 &&
            state.venting_valve_btn == 1)
        {
            // The venting switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            modules.get<Actuators>()->toggleServo(ServosList::VENTING_VALVE);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.start_tars_btn == 0 && state.start_tars_btn == 1)
        {
            // The TARS switch was pressed
            EventBroker::getInstance().post(MOTOR_START_TARS, TOPIC_TARS);

            lastManualActuation = currentTime;
        }
    }

    // Special case for disarming, that can be done bypassing the timeout
    if (oldConrigState.arm_switch == 1 && state.arm_switch == 0)
    {
        EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
        EventBroker::getInstance().post(TMTC_DISARM, TOPIC_MOTOR);

        lastManualActuation = currentTime;
    }

    oldConrigState = state;
}
