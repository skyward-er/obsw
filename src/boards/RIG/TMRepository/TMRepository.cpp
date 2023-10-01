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
#include <RIG/BoardScheduler.h>
#include <RIG/Radio/Radio.h>
#include <RIG/Sensors/Sensors.h>
#include <RIG/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIG/StateMachines/TARS1/TARS1.h>
#include <RIG/StatesMonitor/StatesMonitor.h>
#include <RIG/TMRepository/TMRepository.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

using namespace Boardcore;
using namespace miosix;
using namespace Common;

namespace RIG
{
mavlink_message_t TMRepository::packSystemTm(SystemTMList tmId, uint8_t msgId,
                                             uint8_t seq)
{
    mavlink_message_t msg;
    ModuleManager& modules = ModuleManager::getInstance();

    // Prevents preemption, THIS FUNCTION MUST NOT yeld OR USE KERNEL. THE ONLY
    // ALLOWED OPERATION ARE COPIES OF DATA STRUCTURES.
    PauseKernelLock lock;

    switch (tmId)
    {
        case SystemTMList::MAV_SYS_ID:
        {
            mavlink_sys_tm_t tm;

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.logger          = Logger::getInstance().isStarted();
            tm.event_broker    = EventBroker::getInstance().isRunning();
            tm.radio           = modules.get<Radio>()->isStarted();
            tm.sensors         = modules.get<Sensors>()->isStarted();
            tm.board_scheduler = modules.get<BoardScheduler>()->isStarted();

            mavlink_msg_sys_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            break;
        }
        case SystemTMList::MAV_LOGGER_ID:
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

            break;
        }
        case SystemTMList::MAV_MAVLINK_STATS:
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
            break;
        }
        case SystemTMList::MAV_GSE_ID:
        {
            mavlink_gse_tm_t tm;

            HX711Data tank   = modules.get<Sensors>()->getTankWeight();
            HX711Data vessel = modules.get<Sensors>()->getVesselWeight();

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.loadcell_rocket = tank.load;
            tm.loadcell_vessel = vessel.load;
            tm.filling_pressure =
                modules.get<Sensors>()->getFillingPress().pressure;
            tm.vessel_pressure =
                modules.get<Sensors>()->getVesselPress().pressure;
            // TODO change the threshold (cannot be 0 due to angle tolerance)
            tm.filling_valve_state = modules.get<Actuators>()->getServoPosition(
                                         ServosList::FILLING_VALVE) > 0.3
                                         ? 1
                                         : 0;
            tm.venting_valve_state = modules.get<Actuators>()->getServoPosition(
                                         ServosList::VENTING_VALVE) > 0.3
                                         ? 1
                                         : 0;
            tm.release_valve_state = modules.get<Actuators>()->getServoPosition(
                                         ServosList::RELEASE_VALVE) > 0.3
                                         ? 1
                                         : 0;

            tm.main_valve_state = modules.get<Actuators>()->getServoPosition(
                                      ServosList::MAIN_VALVE) > 0.3
                                      ? 1
                                      : 0;

            tm.arming_state =
                modules.get<GroundModeManager>()->getStatus().state ==
                        GroundModeManagerState::ARMED
                    ? 1
                    : 0;
            tm.ignition_state =
                modules.get<GroundModeManager>()->getStatus().state ==
                        GroundModeManagerState::IGNITING
                    ? 1
                    : 0;
            tm.tars_state =
                static_cast<uint8_t>(modules.get<TARS1>()->getStatus().state);
            tm.battery_voltage =
                modules.get<Sensors>()->getBatteryVoltage().voltage;
            tm.current_consumption =
                modules.get<Sensors>()->getCurrent().current;

            // Board statuses
            tm.main_board_status =
                modules.get<StatesMonitor>()->getStatus(CanConfig::Board::MAIN);
            tm.payload_board_status = modules.get<StatesMonitor>()->getStatus(
                CanConfig::Board::PAYLOAD);
            tm.motor_board_status = modules.get<StatesMonitor>()->getStatus(
                CanConfig::Board::MOTOR);

            mavlink_msg_gse_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                      Config::Radio::MAV_COMPONENT_ID, &msg,
                                      &tm);
            break;
        }
        case SystemTMList::MAV_MOTOR_ID:
        {
            mavlink_motor_tm_t tm;

            TemperatureData temp =
                modules.get<Sensors>()->getThermocoupleLastSample();

            tm.timestamp = TimestampTimer::getTimestamp();
            tm.top_tank_pressure =
                modules.get<Sensors>()->getTankTopPress().pressure;
            tm.bottom_tank_pressure =
                modules.get<Sensors>()->getTankBottomPress().pressure;
            tm.combustion_chamber_pressure =
                modules.get<Sensors>()->getCCPress().pressure;
            tm.tank_temperature = temp.temperature;
            tm.floating_level   = 0;

            tm.battery_voltage =
                modules.get<Sensors>()->getMotorBatteryVoltage().batVoltage;
            tm.current_consumption =
                modules.get<Sensors>()->getMotorCurrent().current;

            tm.main_valve_state = modules.get<Actuators>()->getServoPosition(
                                      ServosList::MAIN_VALVE) > 0.3
                                      ? 1
                                      : 0;

            mavlink_msg_motor_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                        Config::Radio::MAV_COMPONENT_ID, &msg,
                                        &tm);

            break;
        }
        default:
        {
            // No system ID recognized, send a nack
            mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                                     Config::Radio::MAV_COMPONENT_ID, &msg,
                                     msgId, seq);
            break;
        }
    }
    return msg;
}

mavlink_message_t TMRepository::packSensorsTm(SensorsTMList sensorId,
                                              uint8_t msgId, uint8_t seq)
{
    mavlink_message_t msg;
    ModuleManager& modules = ModuleManager::getInstance();

    // NO NEED FOR KERNEL LOCK BECAUSE IT IS ALREADY PRESENT INSIDE THE GETTERS
    switch (sensorId)
    {
        case SensorsTMList::MAV_FILLING_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData sample = modules.get<Sensors>()->getFillingPress();

            tm.timestamp = sample.pressureTimestamp;
            tm.pressure  = sample.pressure;

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_TANK_TOP_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData sample = modules.get<Sensors>()->getTankTopPress();

            tm.timestamp = sample.pressureTimestamp;
            tm.pressure  = sample.pressure;

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_TANK_BOTTOM_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData sample = modules.get<Sensors>()->getTankBottomPress();

            tm.timestamp = sample.pressureTimestamp;
            tm.pressure  = sample.pressure;

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_VESSEL_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData sample = modules.get<Sensors>()->getVesselPress();

            tm.timestamp = sample.pressureTimestamp;
            tm.pressure  = sample.pressure;

            mavlink_msg_pressure_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_TANK_TEMP_ID:
        {
            mavlink_temp_tm_t tm;

            TemperatureData temp =
                modules.get<Sensors>()->getThermocoupleLastSample();

            tm.timestamp   = temp.temperatureTimestamp;
            tm.temperature = temp.temperature;

            mavlink_msg_temp_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);

            break;
        }
        case SensorsTMList::MAV_LOAD_CELL_VESSEL_ID:
        {
            mavlink_load_tm_t tm;

            HX711Data load = modules.get<Sensors>()->getVesselWeight();

            tm.timestamp = load.loadTimestamp;
            tm.load      = load.load;

            mavlink_msg_load_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            break;
        }
        case SensorsTMList::MAV_LOAD_CELL_TANK_ID:
        {
            mavlink_load_tm_t tm;

            HX711Data load = modules.get<Sensors>()->getTankWeight();

            tm.timestamp = load.loadTimestamp;
            tm.load      = load.load;

            mavlink_msg_load_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tm);
            break;
        }
        default:
        {
            // No sensor ID recognized, send a nack
            mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                                     Config::Radio::MAV_COMPONENT_ID, &msg,
                                     msgId, seq);
            break;
        }
    }
    return msg;
}

mavlink_message_t TMRepository::packServoTm(ServosList servoId, uint8_t msgId,
                                            uint8_t seq)
{
    mavlink_message_t msg;
    ModuleManager& modules = ModuleManager::getInstance();

    // Prevents preemption, THIS FUNCTION MUST NOT yeld OR USE KERNEL. THE ONLY
    // ALLOWED OPERATION ARE COPIES OF DATA STRUCTURES.
    PauseKernelLock lock;

    if (servoId != MAIN_VALVE && servoId != VENTING_VALVE &&
        servoId != RELEASE_VALVE && servoId != FILLING_VALVE &&
        servoId != DISCONNECT_SERVO)
    {
        // No servo ID recognized, send a nack
        mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                                 Config::Radio::MAV_COMPONENT_ID, &msg, msgId,
                                 seq);
    }
    else
    {
        mavlink_servo_tm_t tm;

        tm.servo_id       = servoId;
        tm.servo_position = modules.get<Actuators>()->getServoPosition(servoId);

        mavlink_msg_servo_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                    Config::Radio::MAV_COMPONENT_ID, &msg, &tm);
    }

    return msg;
}
}  // namespace RIG