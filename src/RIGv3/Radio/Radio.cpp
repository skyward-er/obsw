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

#include <ConRIGv2/Configs/RadioConfig.h>
#include <common/Events.h>
#include <common/Radio.h>
#include <common/canbus/MotorStatus.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>
#include <utils/Registry/RegistryFrontend.h>

#include <atomic>
#include <chrono>
#include <unordered_map>

using namespace std::chrono;
using namespace Boardcore;
using namespace miosix;
using namespace Common;
using namespace RIGv3;

SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Lora* instance = gRadio;
    if (instance)
        instance->handleDioIRQ();
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
        radio.get(), [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); }, Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize RIG mav driver");
        return false;
    }

    started = true;
    return true;
}

void Radio::enqueueMessage(const mavlink_message_t& msg)
{
    queuedMessages.put(msg);
}

void Radio::flushMessages()
{
    try
    {
        size_t bytesSent = 0;

        while (!queuedMessages.isEmpty())
        {
            auto& message      = queuedMessages.get();
            auto messageLength = mavlink_msg_get_send_buffer_length(&message);
            if (bytesSent + messageLength > Config::Radio::MAX_FLUSH_SIZE)
                break;

            bool enqueued = mavDriver->enqueueMsg(message);
            // If we reached the maximum queue size, stop flushing
            if (!enqueued)
                break;

            queuedMessages.pop();
            bytesSent += messageLength;
        }
    }
    catch (std::range_error& e)
    {
        // This shouldn't happen, but still try to prevent it
        LOG_ERR(logger, "Error while flushing packets: {}", e.what());
    }
}

void Radio::enqueueAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueueMessage(ackMsg);
}

void Radio::enqueueNack(const mavlink_message_t& msg, uint8_t errorId)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq, errorId);
    enqueueMessage(nackMsg);
}

void Radio::enqueueWack(const mavlink_message_t& msg, uint8_t errorId)
{
    mavlink_message_t wackMsg;
    mavlink_msg_wack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &wackMsg,
                             msg.msgid, msg.seq, errorId);
    enqueueMessage(wackMsg);
}

MavlinkStatus Radio::getMavStatus()
{
    if (mavDriver)
        return mavDriver->getStatus();
    else
        return {};
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
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);

            break;
        }

        case MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC:
        {
            uint8_t tmId =
                mavlink_msg_sensor_tm_request_tc_get_sensor_name(&msg);
            if (enqueueSensorTm(tmId))
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);

            break;
        }

        case MAVLINK_MSG_ID_WIGGLE_SERVO_TC:
        {
            ServosList servo = static_cast<ServosList>(
                mavlink_msg_wiggle_servo_tc_get_servo_id(&msg));

            if (getModule<GroundModeManager>()->getState() ==
                GroundModeManagerState::DISARMED)
            {
                if (getModule<Actuators>()->wiggleValve(servo))
                    enqueueAck(msg);
                else
                    enqueueNack(msg, 0);
            }
            else
            {
                enqueueNack(msg, 0);
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
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);
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
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);
            break;
        }

        case MAVLINK_MSG_ID_SET_FIRING_PARAMETERS_TC:
        {
            uint32_t fullThrottleTime =
                mavlink_msg_set_firing_parameters_tc_get_full_throttle_time(
                    &msg);
            uint32_t lowThrottleTime =
                mavlink_msg_set_firing_parameters_tc_get_low_throttle_time(
                    &msg);
            float pilotFlameOxPosition =
                mavlink_msg_set_firing_parameters_tc_get_pilot_flame_ox_position(
                    &msg);
            float pilotFlameFuelPosition =
                mavlink_msg_set_firing_parameters_tc_get_pilot_flame_fuel_position(
                    &msg);
            float lowThrottleOxPosition =
                mavlink_msg_set_firing_parameters_tc_get_low_throttle_ox_position(
                    &msg);
            float lowThrottleFuelPosition =
                mavlink_msg_set_firing_parameters_tc_get_low_throttle_fuel_position(
                    &msg);

            getModule<FiringSequenceHSM>()->setFiringParams(
                fullThrottleTime, lowThrottleTime, pilotFlameOxPosition,
                pilotFlameFuelPosition, lowThrottleOxPosition,
                lowThrottleFuelPosition);

            enqueueAck(msg);
            break;
        }

        case MAVLINK_MSG_ID_SET_PRESSURE_THRESHOLDS_TC:
        {
            float ignPressureThreshold =
                mavlink_msg_set_pressure_thresholds_tc_get_ign_ok_threshold(
                    &msg);

            float pilotPressureThreshold =
                mavlink_msg_set_pressure_thresholds_tc_get_pilot_ok_threshold(
                    &msg);

            getModule<FiringSequenceHSM>()->setPressureThresholds(ignPressureThreshold,
                                                        pilotPressureThreshold);
            enqueueAck(msg);
            break;
        }

        case MAVLINK_MSG_ID_GET_VALVE_INFO_TC:
        {
            ServosList valveId = static_cast<ServosList>(
                mavlink_msg_get_valve_info_tc_get_servo_id(&msg));

            bool result = enqueueValveInfoTm(valveId);
            if (result)
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);

            break;
        }

        case MAVLINK_MSG_ID_SET_TARS3_PARAMS_TC:
        {
            mavlink_set_tars3_params_tc_t params;
            mavlink_msg_set_tars3_params_tc_decode(&msg, &params);

            auto error = getModule<Registry>()->setUnsafe(
                CONFIG_ID_TARS3_MASS_TARGET, params.mass_target);
            if (error != RegistryError::OK)
            {
                enqueueNack(msg, 0);
                break;
            }

            error = getModule<Registry>()->setUnsafe(
                CONFIG_ID_TARS3_PRESSURE_TARGET, params.pressure_target);
            if (error != RegistryError::OK)
            {
                enqueueNack(msg, 0);
                break;
            }

            enqueueAck(msg);
            break;
        }

        case MAVLINK_MSG_ID_SET_EREG_CONSTANTS_TC:
        {
            EregPIDConfig pressurizationConfig = {
                mavlink_msg_set_ereg_constants_tc_get_kp_pressurization(&msg),
                mavlink_msg_set_ereg_constants_tc_get_ki_pressurization(&msg),
                mavlink_msg_set_ereg_constants_tc_get_kd_pressurization(&msg),
            };

            EregPIDConfig dischargeConfig = {
                mavlink_msg_set_ereg_constants_tc_get_kp_discharge(&msg),
                mavlink_msg_set_ereg_constants_tc_get_ki_discharge(&msg),
                mavlink_msg_set_ereg_constants_tc_get_kd_discharge(&msg),
            };

            if (mavlink_msg_set_ereg_constants_tc_get_ereg(&msg) == EREG_OX &&
                getModule<EregControllerOx>()->getState() == EregState::CLOSED)
            {
                getModule<EregControllerOx>()->changePIDConfig(
                    pressurizationConfig, dischargeConfig);
                return enqueueAck(msg);
            }
            if (mavlink_msg_set_ereg_constants_tc_get_ereg(&msg) == EREG_FUEL &&
                getModule<EregControllerFuel>()->getState() ==
                    EregState::CLOSED)
            {
                getModule<EregControllerFuel>()->changePIDConfig(
                    pressurizationConfig, dischargeConfig);
                return enqueueAck(msg);
            }

            return enqueueNack(msg, 0);
        }

        case MAVLINK_MSG_ID_SET_EREG_TARGET_TC:
        {
            float targetPressure =
                mavlink_msg_set_ereg_target_tc_get_pressure_target(&msg);

            if (mavlink_msg_set_ereg_target_tc_get_ereg(&msg) == EREG_OX)
            {
                getModule<EregControllerOx>()->changeTargetPressure(
                    targetPressure);
            }
            else
            {
                getModule<EregControllerFuel>()->changeTargetPressure(
                    targetPressure);
            }

            return enqueueAck(msg);
        }

        case MAVLINK_MSG_ID_RAW_EVENT_TC:
        {
            uint8_t topicId = mavlink_msg_raw_event_tc_get_topic_id(&msg);
            uint8_t eventId = mavlink_msg_raw_event_tc_get_event_id(&msg);

            bool disarmed = getModule<GroundModeManager>()->getState() ==
                            GroundModeManagerState::DISARMED;
            // Raw events are allowed in the disarmed state
            if (!disarmed)
                return enqueueNack(msg, 0);

            EventBroker::getInstance().post(eventId, topicId);
            return enqueueAck(msg);
        }

        default:
        {
            // Unrecognized packet
            enqueueNack(msg, 0);
            break;
        }
    }
}

void Radio::handleCommand(const mavlink_message_t& msg)
{
    uint8_t cmdId = mavlink_msg_command_tc_get_command_id(&msg);
    switch (cmdId)
    {
        case MAV_CMD_START_LOGGING:
        {
            if (Logger::getInstance().start())
            {
                Logger::getInstance().resetStats();
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
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
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);
            break;
        }

        case MAV_CMD_REGISTRY_SAVE:
        {
            if (getModule<Registry>()->save() == RegistryError::OK)
                enqueueAck(msg);
            else
                enqueueNack(msg, 0);
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
            // Try to map the command to an event
            auto ev = mavCmdToEvent(cmdId);
            if (ev != LAST_EVENT)
            {
                EventBroker::getInstance().post(ev, TOPIC_TMTC);
                enqueueAck(msg);
            }
            else
            {
                enqueueNack(msg, 0);
            }
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

            // A safe copy function for key names
            auto copyKeyName = [](auto& dest, const char* src)
            {
                // Ensure dest is not a pointer
                static_assert(sizeof(dest) != sizeof(char*));

                constexpr size_t maxKeyLen = sizeof(dest) / sizeof(dest[0]) - 1;
                std::strncpy(dest, src, maxKeyLen);
                dest[maxKeyLen] = '\0';  // Ensure null-termination
            };

            switch (value.getType())
            {
                case TypesEnum::UINT32:
                {
                    mavlink_registry_int_tm_t tm;

                    tm.timestamp = TimestampTimer::getTimestamp();
                    tm.key_id    = id;
                    copyKeyName(tm.key_name, name);
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
                    copyKeyName(tm.key_name, name);
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
                    copyKeyName(tm.key_name, name);
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

            enqueueMessage(msg);
        });
}

bool Radio::enqueueValveInfoTm(ServosList valveId)
{
    mavlink_message_t msg;
    mavlink_valve_info_tm_t tm;

    auto valveInfo = getModule<Actuators>()->getValveState(valveId);

    if (!valveInfo.valid)
        return false;

    tm.servo_id      = valveId;
    tm.state         = valveInfo.state;
    tm.timing        = milliseconds{valveInfo.timing}.count();
    tm.time_to_close = milliseconds{valveInfo.timeToClose}.count();
    tm.aperture      = valveInfo.aperture * 100;  // Convert to percentage

    mavlink_msg_valve_info_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                     Config::Radio::MAV_COMPONENT_ID, &msg,
                                     &tm);
    enqueueMessage(msg);
    return true;
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
                enqueueMessage(msg);
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
            enqueueMessage(msg);
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
            enqueueMessage(msg);
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
            enqueueMessage(msg);
            return true;
        }

        case MAV_GSE_ID:
        {
            mavlink_message_t msg;
            mavlink_gse_tm_t tm = {0};

            Sensors* sensors     = getModule<Sensors>();
            Actuators* actuators = getModule<Actuators>();
            MotorStatus* motor   = getModule<MotorStatus>();

            tm.timestamp = TimestampTimer::getTimestamp();

            // Sensors
            // tm.rocket_mass    = sensors->getRocketWeight().load;
            tm.ox_vessel_mass = sensors->getOxVesselWeight().load;
            // tm.ox_tank_mass   = sensors->getOxTankWeight().load;

            tm.ox_vessel_pressure = sensors->getOxVesselPressure().pressure;
            tm.prz_vessel_1_pressure =
                sensors->getPrzVessel1Pressure().pressure;
            tm.prz_vessel_2_pressure =
                sensors->getPrzVessel2Pressure().pressure;
            tm.ox_filling_pressure  = sensors->getPrzFillingPressure().pressure;
            tm.prz_filling_pressure = sensors->getPrzFillingPressure().pressure;

            tm.battery_voltage     = sensors->getBatteryVoltage().voltage;
            tm.current_consumption = sensors->getServoCurrent().current;
            tm.umbilical_current_consumption =
                sensors->getUmbilicalCurrent().current;

            // Log data
            LoggerStats stats = sdLogger.getStats();
            tm.log_number     = stats.logNumber;
            tm.log_good       = stats.lastWriteError == 0;

            // Cpu stuff
            CpuMeterData cpuStats = CpuMeter::getCpuStats();
            CpuMeter::resetCpuStats();
            tm.cpu_load  = cpuStats.mean;
            tm.free_heap = cpuStats.freeHeap;

            // Also log this to the SD
            sdLogger.log(cpuStats);

            // Valve states
            tm.ox_filling_valve_state =
                actuators->isValveOpen(ServosList::OX_FILLING_VALVE);
            tm.ox_release_valve_state =
                actuators->isValveOpen(ServosList::OX_RELEASE_VALVE);
            tm.ox_detach_state =
                actuators->isValveOpen(ServosList::OX_DETACH_SERVO);
            tm.prz_filling_valve_state =
                actuators->isValveOpen(ServosList::PRZ_FILLING_VALVE);
            tm.prz_release_valve_state =
                actuators->isValveOpen(ServosList::PRZ_RELEASE_VALVE);
            tm.prz_detach_state =
                actuators->isValveOpen(ServosList::PRZ_DETACH_SERVO);
            tm.prz_3way_valve_state = actuators->get3wayValveState();

            // Internal states
            tm.gmm_state    = getModule<GroundModeManager>()->getState();
            tm.tars1_state  = (uint8_t)getModule<TARS1>()->getLastAction();
            tm.tars3_state  = (uint8_t)getModule<TARS3>()->getLastAction();
            tm.arming_state = getModule<GroundModeManager>()->getState() ==
                              GroundModeManagerState::ARMED;

            // Can data
            CanHandler::CanStatus canStatus =
                getModule<CanHandler>()->getCanStatus();
            tm.main_board_state  = canStatus.getMainState();
            tm.motor_board_state = motor->getState();
            tm.main_can_status   = canStatus.isMainConnected();
            tm.motor_can_status  = motor->connected();

            mavlink_msg_gse_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_MOTOR_ID:
        {
            auto motor = getModule<MotorStatus>();

            mavlink_message_t msg;
            mavlink_motor_tm_t tm = motor->getMotorTelemetry();

            // Use RIG data if motor is not detected
            if (!motor->detected())
            {
                auto sensors          = getModule<Sensors>();
                tm.prz_tank_pressure  = sensors->getPrzTankPressure().pressure;
                tm.ox_tank_pressure   = sensors->getOxTankPressure().pressure;
                tm.fuel_tank_pressure = sensors->getFuelTankPressure().pressure;
                tm.ox_reg_out_pressure =
                    sensors->getOxRegOutPressure().pressure;
                tm.fuel_reg_out_pressure =
                    sensors->getFuelRegOutPressure().pressure;
                tm.main_cc_pressure =
                    sensors->getMainChamberPressure().pressure;
                tm.ign_cc_pressure =
                    sensors->getIgniterChamberPressure().pressure;

                // valve states
                tm.ox_venting_valve_state = getModule<Actuators>()->isValveOpen(
                    ServosList::OX_VENTING_VALVE);
                tm.fuel_venting_valve_state =
                    getModule<Actuators>()->isValveOpen(
                        ServosList::FUEL_VENTING_VALVE);

                tm.prz_ox_valve_state =
                    (getModule<EregControllerOx>()->getState() !=
                     EregState::CLOSED);
                tm.prz_fuel_valve_state =
                    (getModule<EregControllerFuel>()->getState() !=
                     EregState::CLOSED);

                tm.prz_ox_valve_position =
                    static_cast<uint8_t>(sensors->getOxRegPosition().position);
                tm.prz_fuel_valve_position = static_cast<uint8_t>(
                    sensors->getFuelRegPosition().position);

                tm.main_ox_valve_state = getModule<Actuators>()->isValveOpen(
                    ServosList::MAIN_OX_VALVE);
                tm.main_fuel_valve_state = getModule<Actuators>()->isValveOpen(
                    ServosList::MAIN_FUEL_VALVE);

                tm.main_fuel_valve_position = static_cast<uint8_t>(
                    sensors->getMainFuelPosition().position);
                tm.main_ox_valve_position =
                    static_cast<uint8_t>(sensors->getMainOxPosition().position);

                tm.ox_solenoid_state = getModule<Actuators>()->isValveOpen(
                    ServosList::IGNITION_OX_VALVE);
                tm.fuel_solenoid_state = getModule<Actuators>()->isValveOpen(
                    ServosList::IGNITION_FUEL_VALVE);
                tm.spark_igniter_state =
                    getModule<Actuators>()->isSparkSparking();

                tm.firing_sequence_hsm_state =
                    (uint8_t)getModule<FiringSequenceHSM>()->getState();
            }
            mavlink_msg_motor_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                        Config::Radio::MAV_COMPONENT_ID, &msg,
                                        &tm);
            enqueueMessage(msg);
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
            strcpy(tm.sensor_name, "ADS131M08_1");

            mavlink_msg_adc_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueueMessage(msg);

            data = getModule<Sensors>()->getADC2LastSample();

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
            strcpy(tm.sensor_name, "ADS131M08_2");

            mavlink_msg_adc_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            enqueueMessage(msg);

            return true;
        }

        case MAV_VESSEL_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            PressureData data = getModule<Sensors>()->getOxVesselPressure();

            tm.timestamp = data.pressureTimestamp;
            tm.pressure  = data.pressure;
            strcpy(tm.sensor_name, "OxVesselPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_TANK_TOP_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            {
                auto motor   = getModule<MotorStatus>()->lockData();
                auto sample  = motor->oxTankTopPressure;
                tm.pressure  = sample.pressure;
                tm.timestamp = sample.pressureTimestamp;
            }
            strcpy(tm.sensor_name, "TopTankPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueueMessage(msg);

            return true;
        }

        case MAV_TANK_BOTTOM_PRESS_ID:
        {
            PressureData bottom0;
            PressureData bottom1;

            {
                auto motor = getModule<MotorStatus>()->lockData();
                bottom0    = motor->oxTankBottom0Pressure;
                bottom1    = motor->oxTankBottom1Pressure;
            }

            {
                mavlink_message_t msg;
                mavlink_pressure_tm_t tm;

                tm.pressure  = bottom0.pressure;
                tm.timestamp = bottom0.pressureTimestamp;
                strcpy(tm.sensor_name, "TankBottom0Pressure");

                mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                               Config::Radio::MAV_COMPONENT_ID,
                                               &msg, &tm);
                enqueueMessage(msg);
            }
            {
                mavlink_message_t msg;
                mavlink_pressure_tm_t tm;

                tm.pressure  = bottom1.pressure;
                tm.timestamp = bottom1.pressureTimestamp;
                strcpy(tm.sensor_name, "TankBottom1Pressure");

                mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                               Config::Radio::MAV_COMPONENT_ID,
                                               &msg, &tm);
                enqueueMessage(msg);
            }

            return true;
        }

        case MAV_COMBUSTION_PRESS_ID:
        {
            mavlink_message_t msg;
            mavlink_pressure_tm_t tm;

            {
                auto motor   = getModule<MotorStatus>()->lockData();
                auto sample  = motor->combustionChamberPressure;
                tm.pressure  = sample.pressure;
                tm.timestamp = sample.pressureTimestamp;
            }
            strcpy(tm.sensor_name, "CCPressure");

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);
            enqueueMessage(msg);

            return true;
        }

        case MAV_LOAD_CELL_VESSEL_ID:
        {
            mavlink_message_t msg;
            mavlink_load_tm_t tm;

            LoadCellData data = getModule<Sensors>()->getOxVesselWeight();

            tm.timestamp = data.loadTimestamp;
            tm.load      = data.load;
            strcpy(tm.sensor_name, "OxVesselWeight");

            mavlink_msg_load_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_LOAD_CELL_TANK_ID:
        {
            mavlink_message_t msg;
            mavlink_load_tm_t tm;

            // TODO: this has been removed for now
            LoadCellData data{} /* = getModule<Sensors>()->getRocketWeight() */;

            tm.timestamp = data.loadTimestamp;
            tm.load      = data.load;
            strcpy(tm.sensor_name, "OxTankWeight");

            mavlink_msg_load_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            enqueueMessage(msg);
            return true;
        }

        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_message_t msg;
            mavlink_voltage_tm_t tm;

            VoltageData data = getModule<Sensors>()->getBatteryVoltage();

            tm.timestamp = data.voltageTimestamp;
            tm.voltage   = data.voltage;
            strcpy(tm.sensor_name, "BatteryVoltage");

            mavlink_msg_voltage_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                          Config::Radio::MAV_COMPONENT_ID, &msg,
                                          &tm);
            enqueueMessage(msg);
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
    mavlink_conrig_state_tc_t state;
    mavlink_msg_conrig_state_tc_decode(&msg, &state);

#define BUTTON_PRESSED(btn) (lastConrigState.btn == 0 && state.btn == 1)
#define SWITCH_CHANGED(sw) (lastConrigState.sw != state.sw)

    // Use a debounce time to prevent updates too close to each other
    // that may be caused by interference on the buttons
    auto debounceTime     = msToNs(Config::Radio::LAST_COMMAND_THRESHOLD);
    long long currentTime = getTime();

    if (currentTime > lastManualActuation + debounceTime)
    {
        // Ok we can accept new commands
        if (BUTTON_PRESSED(arm_switch))
        {
            // The ARM switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            EventBroker::getInstance().post(TMTC_ARM, TOPIC_MOTOR);
            lastManualActuation = currentTime;
        }

        if (BUTTON_PRESSED(ignition_btn))
        {
            // The ignition switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            EventBroker::getInstance().post(MOTOR_IGNITION, TOPIC_MOTOR);
            lastManualActuation = currentTime;
        }

        if (BUTTON_PRESSED(ox_filling_btn))
        {
            // The OX filling switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(ServosList::OX_FILLING_VALVE);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::OX_FILLING_VALVE);
        }

        if (BUTTON_PRESSED(ox_release_btn))
        {
            // The OX release switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(ServosList::OX_RELEASE_VALVE);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::OX_RELEASE_VALVE);
        }

        if (BUTTON_PRESSED(prz_filling_btn))
        {
            // The PRZ filling switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(ServosList::PRZ_FILLING_VALVE);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::PRZ_FILLING_VALVE);
        }

        if (BUTTON_PRESSED(prz_release_btn))
        {
            // The PRZ release switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(ServosList::PRZ_RELEASE_VALVE);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::PRZ_RELEASE_VALVE);
        }

        if (BUTTON_PRESSED(prz_ox_btn))
        {
            // The OX pressurize switch was pressed
            EventBroker::getInstance().post(EREG_TOGGLE, TOPIC_EREG_OX);
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::PRZ_OX_VALVE);
        }

        if (BUTTON_PRESSED(prz_fuel_btn))
        {
            // The OX pressurize switch was pressed
            EventBroker::getInstance().post(EREG_TOGGLE, TOPIC_EREG_FUEL);
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::PRZ_FUEL_VALVE);
        }

        if (BUTTON_PRESSED(ox_venting_btn))
        {
            // The OX venting switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(ServosList::OX_VENTING_VALVE);
            lastManualActuation = currentTime;
            enqueueValveInfoTm(ServosList::OX_VENTING_VALVE);
        }

        if (BUTTON_PRESSED(detach_btn))
        {
            // The detach switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(ServosList::OX_DETACH_SERVO);
            getModule<Actuators>()->toggleValve(ServosList::PRZ_DETACH_SERVO);
            lastManualActuation = currentTime;
        }

        if (BUTTON_PRESSED(spare_1_btn))
        {
            // The detach switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);

            lastManualActuation = currentTime;
        }

        if (BUTTON_PRESSED(spare_2_btn))
        {
            // The detach switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->toggleValve(
                ServosList::IGNITION_FUEL_VALVE);
            lastManualActuation = currentTime;
        }

        if (SWITCH_CHANGED(tars_switch))
        {
            // The TARS switch changed state
            switch (state.tars_switch)
            {
                case TARSList::TARS_OFF:
                    EventBroker::getInstance().post(MOTOR_STOP_TARS,
                                                    TOPIC_TMTC);
                    break;

                case TARSList::TARS_1:
                    EventBroker::getInstance().post(MOTOR_START_TARS1,
                                                    TOPIC_TMTC);
                    break;

                case TARSList::TARS_3:
                    EventBroker::getInstance().post(MOTOR_START_TARS3,
                                                    TOPIC_TMTC);
                    break;
            }

            lastManualActuation = currentTime;
        }

        if (SWITCH_CHANGED(prz_3way_switch))
        {
            // The 3-way valve switch was pressed
            EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
            getModule<Actuators>()->set3wayValveState(state.prz_3way_switch);
            lastManualActuation = currentTime;
        }
    }

    // Special case for disarming, that can be done bypassing the timeout
    if (lastConrigState.arm_switch == 1 && state.arm_switch == 0)
    {
        EventBroker::getInstance().post(MOTOR_MANUAL_ACTION, TOPIC_TARS);
        EventBroker::getInstance().post(TMTC_DISARM, TOPIC_MOTOR);

        lastManualActuation = currentTime;
    }

    // Send GSE and motor telemetry
    enqueueSystemTm(MAV_GSE_ID);
    enqueueSystemTm(MAV_MOTOR_ID);
    // Acknowledge the state
    enqueueAck(msg);

    // Flush all pending packets
    flushMessages();

    lastConrigState = state;
}
