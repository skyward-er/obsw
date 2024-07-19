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

#include <common/Events.h>
#include <common/Radio.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>
// TODO(davide.mor): Remove TimestampTimer
#include <drivers/timer/TimestampTimer.h>

#include <atomic>
#include <unordered_map>

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
    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txEn::getPin(),
                                        radio::rxEn::getPin());

    // Setup transceiver
    radio = std::make_unique<SX1278Lora>(
        getModule<Buses>()->getRadio(), radio::cs::getPin(),
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
    size_t count =
        std::min(queuedPackets.count(), Config::Radio::MAX_PACKETS_PER_FLUSH);
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

void Radio::enqueueAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueuePacket(ackMsg);
}

void Radio::enqueueNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq, 0);
    enqueuePacket(nackMsg);
}

Boardcore::MavlinkStatus Radio::getMavStatus()
{
    if (mavDriver)
    {
        return mavDriver->getStatus();
    }
    else
    {
        return {};
    }
}

void Radio::handleMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PING_TC:
        {
            enqueueAck(msg);
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
            if (enqueueSystemTm(tmId))
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }

            break;
        }

        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            uint8_t tmId = mavlink_msg_system_tm_request_tc_get_tm_id(&msg);
            if (enqueueSensorTm(tmId))
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }

            break;
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            if (getModule<GroundModeManager>()->getState() ==
                GMM_STATE_DISARMED)
            {
                if (getModule<Actuators>()->wiggleServo(servo))
                {
                    enqueueAck(msg);
                }
                else
                {
                    enqueueNack(msg);
                }
            }
            else
            {
                enqueueNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC:
        {
            uint32_t time =
                mavlink_msg_set_atomic_valve_timing_tc_get_maximum_timing(&msg);
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_set_atomic_valve_timing_tc_get_servo_id(&msg));

            if (getModule<Actuators>()->setOpeningTime(servo, time))
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
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

            if (getModule<Actuators>()->setMaxAperture(servo, aperture))
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }
            break;
        }

        case MAVLINK_MSG_ID_SET_IGNITION_TIME_TC:
        {
            uint32_t timing = mavlink_msg_set_ignition_time_tc_get_timing(&msg);
            getModule<GroundModeManager>()->setIgnitionTime(timing);

            enqueueAck(msg);
            break;
        }

        default:
        {
            // Unrecognized packet
            enqueueNack(msg);
            break;
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    static const std::unordered_map<uint8_t, Events> cmdToEvent{
        {MAV_CMD_CALIBRATE, TMTC_CALIBRATE},
        {MAV_CMD_FORCE_INIT, TMTC_FORCE_INIT},
        {MAV_CMD_FORCE_REBOOT, TMTC_RESET_BOARD},
        {MAV_CMD_OPEN_NITROGEN, TMTC_OPEN_NITROGEN},
    };

    uint8_t cmd = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmd)
    {
        case MAV_CMD_START_LOGGING:
        {
            if (!Logger::getInstance().start())
            {
                enqueueNack(msg);
            }
            else
            {
                Logger::getInstance().resetStats();
                enqueueAck(msg);
            }
            break;
        }

        case MAV_CMD_STOP_LOGGING:
        {
            Logger::getInstance().stop();
            enqueueAck(msg);
            break;
        }

        case MAV_CMD_REGISTRY_LOAD:
        {
            if (getModule<Registry>()->load() == RegistryError::OK)
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }
            break;
        }

        case MAV_CMD_REGISTRY_SAVE:
        {
            if (getModule<Registry>()->save() == RegistryError::OK)
            {
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg);
            }
            break;
        }

        case MAV_CMD_REGISTRY_CLEAR:
        {
            getModule<Registry>()->clear();
            enqueueAck(msg);
            break;
        }

        default:
        {
            auto it = cmdToEvent.find(cmd);
            if (it != cmdToEvent.end())
            {
                EventBroker::getInstance().post(it->second, TOPIC_MOTOR);
                enqueueAck(msg);
            }
            else
            {
                // Unrecognized command
                enqueueNack(msg);
            }

            break;
        }
    }
}

void Radio::enqueueRegistry()
{
    getModule<Registry>()->forEach(
        [this](ConfigurationId id, EntryStructsUnion& value)
        {
            mavlink_message_t msg;
            const char* name = configurationIdToName(id);

            switch (value.getType())
            {
                case TypesEnum::UINT32:
                {
                    mavlink_registry_int_tm_t tm;

                    tm.timestamp = TimestampTimer::getTimestamp();
                    tm.key_id    = id;
                    strcpy(tm.key_name, name);
                    value.get(tm.value);

                    mavlink_msg_registry_int_tm_encode(
                        Config::Radio::MAV_SYSTEM_ID,
                        Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
                    break;
                }
                case TypesEnum::FLOAT:
                {
                    mavlink_registry_float_tm_t tm;

                    tm.timestamp = TimestampTimer::getTimestamp();
                    tm.key_id    = id;
                    strcpy(tm.key_name, name);
                    value.get(tm.value);

                    mavlink_msg_registry_float_tm_encode(
                        Config::Radio::MAV_SYSTEM_ID,
                        Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
                    break;
                }
                case TypesEnum::COORDINATES:
                {
                    mavlink_registry_coord_tm_t tm;

                    tm.timestamp = TimestampTimer::getTimestamp();
                    tm.key_id    = id;
                    strcpy(tm.key_name, name);
                    Coordinates coord;
                    value.get(coord);
                    tm.latitude  = coord.latitude;
                    tm.longitude = coord.longitude;

                    mavlink_msg_registry_coord_tm_encode(
                        Config::Radio::MAV_SYSTEM_ID,
                        Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
                    break;
                }
            }

            enqueuePacket(msg);
        });
}

bool Radio::enqueueSystemTm(uint8_t tmId)
{
    switch (tmId)
    {
        case MAV_SENSORS_STATE_ID:
        {
            auto sensors = getModule<Sensors>()->getSensorInfos();
            for (auto sensor : sensors)
            {
                mavlink_message_t msg;
                mavlink_sensor_state_tm_t tm;

                strcpy(tm.sensor_name, sensor.id.c_str());
                tm.initialized = sensor.isInitialized ? 1 : 0;
                tm.enabled     = sensor.isEnabled ? 1 : 0;

                mavlink_msg_sensor_state_tm_encode(
                    Config::Radio::MAV_SYSTEM_ID,
                    Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
                enqueuePacket(msg);
            }

            return true;
        }

        case MAV_REGISTRY_ID:
        {
            enqueueRegistry();
            return true;
        }

        case MAV_SYS_ID:
        {
            mavlink_message_t msg;
            mavlink_sys_tm_t tm;

            tm.timestamp    = TimestampTimer::getTimestamp();
            tm.logger       = Logger::getInstance().isStarted() ? 1 : 0;
            tm.event_broker = EventBroker::getInstance().isRunning() ? 1 : 0;
            tm.radio        = isStarted() ? 1 : 0;
            tm.sensors      = getModule<Sensors>()->isStarted() ? 1 : 0;
            tm.actuators    = getModule<Actuators>()->isStarted() ? 1 : 0;
            tm.pin_handler  = 0;  // No pin_handler
            tm.can_handler  = getModule<CanHandler>()->isStarted() ? 1 : 0;
            tm.scheduler    = getModule<BoardScheduler>()->isStarted() ? 1 : 0;

            mavlink_msg_sys_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_LOGGER_ID:
        {
            mavlink_message_t msg;
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
            enqueuePacket(msg);
            return true;
        }

        case MAV_MAVLINK_STATS_ID:
        {
            mavlink_message_t msg;
            mavlink_mavlink_stats_tm_t tm;

            MavlinkStatus stats = mavDriver->getStatus();

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
            enqueuePacket(msg);
            return true;
        }

        case MAV_GSE_ID:
        {
            mavlink_message_t msg;
            mavlink_gse_tm_t tm = {0};

            Sensors* sensors     = getModule<Sensors>();
            Actuators* actuators = getModule<Actuators>();

            tm.timestamp = TimestampTimer::getTimestamp();

            // Sensors
            tm.loadcell_rocket     = sensors->getTankWeight().load;
            tm.loadcell_vessel     = sensors->getVesselWeight().load;
            tm.filling_pressure    = sensors->getFillingPress().pressure;
            tm.vessel_pressure     = sensors->getVesselPress().pressure;
            tm.battery_voltage     = sensors->getBatteryVoltage().voltage;
            tm.current_consumption = sensors->getServoCurrent().current;
            tm.umbilical_current_consumption =
                sensors->getUmbilicalCurrent().current;

            // Log data
            LoggerStats stats = sdLogger.getStats();
            tm.log_number     = stats.logNumber;
            tm.log_good       = stats.lastWriteError == 0;

            // Valve states
            tm.filling_valve_state =
                actuators->isServoOpen(ServosList::FILLING_VALVE) ? 1 : 0;
            tm.venting_valve_state =
                actuators->isServoOpen(ServosList::VENTING_VALVE) ? 1 : 0;
            tm.release_valve_state =
                actuators->isServoOpen(ServosList::RELEASE_VALVE) ? 1 : 0;
            tm.main_valve_state =
                actuators->isServoOpen(ServosList::MAIN_VALVE) ? 1 : 0;
            tm.nitrogen_valve_state = actuators->isNitrogenOpen() ? 1 : 0;

            // Internal states
            tm.gmm_state  = getModule<GroundModeManager>()->getState();
            tm.tars_state = getModule<TARS1>()->isRefueling() ? 1 : 0;
            tm.arming_state =
                getModule<GroundModeManager>()->getState() == GMM_STATE_ARMED
                    ? 1
                    : 0;

            // Can data
            CanHandler::CanStatus canStatus =
                getModule<CanHandler>()->getCanStatus();
            tm.main_board_state    = canStatus.getMainState();
            tm.payload_board_state = canStatus.getPayloadState();
            tm.motor_board_state   = canStatus.getMotorState();
            tm.main_can_status     = canStatus.isMainConnected() ? 1 : 0;
            tm.payload_can_status  = canStatus.isPayloadConnected() ? 1 : 0;
            tm.motor_can_status    = canStatus.isMotorConnected() ? 1 : 0;

            mavlink_msg_gse_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_MOTOR_ID:
        {
            mavlink_message_t msg;
            mavlink_motor_tm_t tm = {0};

            Sensors* sensors     = getModule<Sensors>();
            Actuators* actuators = getModule<Actuators>();

            tm.timestamp = TimestampTimer::getTimestamp();

            // Sensors (either CAN or local)
            tm.top_tank_pressure    = sensors->getTopTankPress().pressure;
            tm.bottom_tank_pressure = sensors->getBottomTankPress().pressure;
            tm.combustion_chamber_pressure = sensors->getCCPress().pressure;
            tm.tank_temperature = sensors->getTc1LastSample().temperature;
            tm.battery_voltage  = sensors->getMotorBatteryVoltage().voltage;

            // Valve states
            tm.main_valve_state =
                actuators->isCanServoOpen(ServosList::MAIN_VALVE) ? 1 : 0;
            tm.venting_valve_state =
                actuators->isCanServoOpen(ServosList::VENTING_VALVE) ? 1 : 0;

            // Can data
            CanHandler::CanStatus canStatus =
                getModule<CanHandler>()->getCanStatus();

            tm.log_number = canStatus.getMotorLogNumber();
            tm.log_good   = canStatus.isMotorLogGood() ? 1 : 0;
            tm.hil_state  = canStatus.isMotorHil() ? 1 : 0;

            mavlink_msg_motor_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                        Config::Radio::MAV_COMPONENT_ID, &msg,
                                        &tm);
            enqueuePacket(msg);
            return true;
        }

        default:
        {
            return false;
        }
    }
}

bool Radio::enqueueSensorTm(uint8_t tmId)
{
    switch (tmId)
    {
        case MAV_ADS131M08_ID:
        {
            mavlink_message_t msg;
            mavlink_adc_tm_t tm;

            ADS131M08Data data = getModule<Sensors>()->getADC1LastSample();

            tm.channel_0 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_0).voltage;
            tm.channel_1 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_1).voltage;
            tm.channel_2 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_2).voltage;
            tm.channel_3 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_3).voltage;
            tm.channel_4 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_4).voltage;
            tm.channel_5 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_5).voltage;
            tm.channel_6 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_6).voltage;
            tm.channel_7 =
                data.getVoltage(ADS131M08Defs::Channel::CHANNEL_7).voltage;
            tm.timestamp = data.timestamp;
            strcpy(tm.sensor_name, "ADS131M08");

            mavlink_msg_adc_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_VESSEL_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            PressureData data = getModule<Sensors>()->getVesselPress();

            tm.timestamp = data.pressureTimestamp;
            tm.pressure  = data.pressure;
            strcpy(tm.sensor_name, "VESSEL_PRESS");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_FILLING_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            PressureData data = getModule<Sensors>()->getFillingPress();

            tm.timestamp = data.pressureTimestamp;
            tm.pressure  = data.pressure;
            strcpy(tm.sensor_name, "FILLING_PRESS");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_TANK_BOTTOM_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            PressureData data = getModule<Sensors>()->getBottomTankPress();

            tm.timestamp = data.pressureTimestamp;
            tm.pressure  = data.pressure;
            strcpy(tm.sensor_name, "BOTTOM_TANK_PRESS");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_TANK_TOP_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            PressureData data = getModule<Sensors>()->getTopTankPress();

            tm.timestamp = data.pressureTimestamp;
            tm.pressure  = data.pressure;
            strcpy(tm.sensor_name, "TOP_TANK_PRESS");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_TANK_TEMP_ID:
        {
            mavlink_message_t msg;
            mavlink_temp_tm_t tm;

            TemperatureData data = getModule<Sensors>()->getTc1LastSample();

            tm.timestamp   = data.temperatureTimestamp;
            tm.temperature = data.temperature;
            strcpy(tm.sensor_name, "MAX31856");

            mavlink_msg_temp_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_LOAD_CELL_VESSEL_ID:
        {
            mavlink_message_t msg;
            mavlink_load_tm_t tm;

            LoadCellData data = getModule<Sensors>()->getVesselWeight();

            tm.timestamp = data.loadTimestamp;
            tm.load      = data.load;
            strcpy(tm.sensor_name, "VESSEL_WEIGHT");

            mavlink_msg_load_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_LOAD_CELL_TANK_ID:
        {
            mavlink_message_t msg;
            mavlink_load_tm_t tm;

            LoadCellData data = getModule<Sensors>()->getTankWeight();

            tm.timestamp = data.loadTimestamp;
            tm.load      = data.load;
            strcpy(tm.sensor_name, "TANK_WEIGHT");

            mavlink_msg_load_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            enqueuePacket(msg);
            return true;
        }

        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_message_t msg;
            mavlink_voltage_tm_t tm;

            VoltageData data = getModule<Sensors>()->getBatteryVoltage();

            tm.timestamp = data.voltageTimestamp;
            tm.voltage   = data.voltage;
            strcpy(tm.sensor_name, "TANK_WEIGHT");

            mavlink_msg_voltage_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
            enqueuePacket(msg);
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
    // Acknowledge the state
    enqueueAck(msg);

    // Flush all pending packets
    flushPackets();

    // Send GSE and motor telemetry
    enqueueSystemTm(MAV_GSE_ID);
    enqueueSystemTm(MAV_MOTOR_ID);

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
            getModule<Actuators>()->toggleServo(ServosList::FILLING_VALVE);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.quick_connector_btn == 0 &&
            state.quick_connector_btn == 1)
        {
            // The quick conector switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleServo(ServosList::DISCONNECT_SERVO);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.release_pressure_btn == 0 &&
            state.release_pressure_btn == 1)
        {
            // The release switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleServo(ServosList::RELEASE_VALVE);

            lastManualActuation = currentTime;
        }

        if (oldConrigState.venting_valve_btn == 0 &&
            state.venting_valve_btn == 1)
        {
            // The venting switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleServo(ServosList::VENTING_VALVE);

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
