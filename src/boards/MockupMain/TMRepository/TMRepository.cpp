/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Angelo Prete
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

#include <MockupMain/BoardScheduler.h>
#include <MockupMain/Configs/RadioConfig.h>
#include <MockupMain/PinHandler/PinHandler.h>
#include <MockupMain/Radio/Radio.h>
#include <MockupMain/Sensors/Sensors.h>
#include <MockupMain/StateMachines/FlightModeManager/FlightModeManager.h>
#include <MockupMain/StateMachines/NASController/NASController.h>
#include <MockupMain/TMRepository/TMRepository.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;

namespace MockupMain
{
mavlink_message_t TMRepository::packSystemTm(SystemTMList tmId, uint8_t msgId,
                                             uint8_t seq)
{
    ModuleManager& modules = ModuleManager::getInstance();
    mavlink_message_t msg;

    switch (tmId)
    {
        case SystemTMList::MAV_SYS_ID:
        {
            mavlink_sys_tm_t tm;

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.logger          = Logger::getInstance().isStarted();
            tm.board_scheduler = modules.get<BoardScheduler>()->isStarted();
            tm.event_broker    = EventBroker::getInstance().isRunning();
            tm.radio           = modules.get<Radio>()->isStarted();
            tm.sensors         = modules.get<Sensors>()->isStarted();
            tm.pin_observer    = modules.get<PinHandler>()->isStarted();

            mavlink_msg_sys_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_LOGGER_ID:
        {
            mavlink_logger_tm_t tm;

            // Get the logger stats
            LoggerStats stats = Logger::getInstance().getStats();

            tm.timestamp          = stats.timestamp;
            tm.average_write_time = stats.averageWriteTime;
            tm.buffers_filled     = stats.buffersFilled;
            tm.buffers_written    = stats.buffersWritten;
            tm.dropped_samples    = stats.droppedSamples;
            tm.last_write_error   = stats.lastWriteError;
            tm.log_number         = stats.logNumber;
            tm.max_write_time     = stats.maxWriteTime;
            tm.queued_samples     = stats.queuedSamples;
            tm.too_large_samples  = stats.tooLargeSamples;
            tm.writes_failed      = stats.writesFailed;

            mavlink_msg_logger_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                         RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_MAVLINK_STATS:
        {
            mavlink_mavlink_stats_tm_t tm;

            // Get the mavlink stats
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

            mavlink_msg_mavlink_stats_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                RadioConfig::MAV_COMP_ID, &msg,
                                                &tm);
            break;
        }
        case SystemTMList::MAV_NAS_ID:
        {
            mavlink_nas_tm_t tm;

            // Get the current NAS state
            NASState state = modules.get<NASController>()->getNasState();
            ReferenceValues ref =
                modules.get<NASController>()->getReferenceValues();

            tm.timestamp = state.timestamp;
            tm.state     = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);
            tm.nas_n           = state.n;
            tm.nas_e           = state.e;
            tm.nas_d           = state.d;
            tm.nas_vn          = state.vn;
            tm.nas_ve          = state.ve;
            tm.nas_vd          = state.vd;
            tm.nas_qx          = state.qx;
            tm.nas_qy          = state.qy;
            tm.nas_qz          = state.qz;
            tm.nas_qw          = state.qw;
            tm.nas_bias_x      = state.bx;
            tm.nas_bias_y      = state.by;
            tm.nas_bias_z      = state.bz;
            tm.ref_pressure    = ref.refPressure;
            tm.ref_temperature = ref.refTemperature;
            tm.ref_latitude    = ref.refLatitude;
            tm.ref_longitude   = ref.refLongitude;

            mavlink_msg_nas_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            mavlink_payload_flight_tm_t tm;

            tm.timestamp = TimestampTimer::getTimestamp();

            // Last samples
            auto lps22df = modules.get<Sensors>()->getLPS22LastSample();
            BMX160Data imu =
                modules.get<Sensors>()->getBMX160WithCorrectionLastSample();
            UBXGPSData gps = modules.get<Sensors>()->getUbxGpsLastSample();
            Eigen::Vector2f wind{};

            // NAS state
            NASState nasState = modules.get<NASController>()->getNasState();

            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            tm.wes_state = 0;

            tm.wes_n = modules.get<Sensors>()->getLoadCellLastSample().load;
            tm.wes_e = modules.get<Sensors>()
                           ->getADS131LastSample()
                           .getVoltage(SensorsConfig::LOAD_CELL_ADC_CHANNEL)
                           .voltage;

            tm.pressure_digi = lps22df.pressure;

            // Altitude agl
            tm.altitude_agl = -nasState.d;

            // IMU
            tm.acc_x  = imu.accelerationX;
            tm.acc_y  = imu.accelerationY;
            tm.acc_z  = imu.accelerationZ;
            tm.gyro_x = imu.angularSpeedX;
            tm.gyro_y = imu.angularSpeedY;
            tm.gyro_z = imu.angularSpeedZ;

            // Magnetometer
            tm.mag_x = imu.magneticFieldX;
            tm.mag_y = imu.magneticFieldY;
            tm.mag_z = imu.magneticFieldZ;

            // GPS
            tm.gps_fix = gps.fix;
            tm.gps_lat = gps.latitude;
            tm.gps_lon = gps.longitude;
            tm.gps_alt = gps.height;

            // NAS
            tm.nas_n      = nasState.n;
            tm.nas_e      = nasState.e;
            tm.nas_d      = nasState.d;
            tm.nas_vn     = nasState.vn;
            tm.nas_ve     = nasState.ve;
            tm.nas_vd     = nasState.vd;
            tm.nas_qx     = nasState.qx;
            tm.nas_qy     = nasState.qy;
            tm.nas_qz     = nasState.qz;
            tm.nas_qw     = nasState.qw;
            tm.nas_bias_x = nasState.bx;
            tm.nas_bias_y = nasState.by;
            tm.nas_bias_z = nasState.bz;

            // Servos
            tm.left_servo_angle  = 0;
            tm.right_servo_angle = 0;

            tm.pin_nosecone = modules.get<PinHandler>()
                                  ->getPinsData()[PinsList::NOSECONE_PIN]
                                  .lastState;

            tm.battery_voltage = modules.get<Sensors>()
                                     ->getBatteryVoltageLastSample()
                                     .batVoltage;
            tm.current_consumption = 0;
            tm.temperature         = lps22df.temperature;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;

            tm.battery_voltage = modules.get<Sensors>()
                                     ->getBatteryVoltageLastSample()
                                     .batVoltage;

            mavlink_msg_payload_flight_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                 RadioConfig::MAV_COMP_ID, &msg,
                                                 &tm);

            break;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_payload_stats_tm_t tm;

            tm.cpu_load  = CpuMeter::getCpuStats().mean;
            tm.free_heap = CpuMeter::getCpuStats().freeHeap;

            mavlink_msg_payload_stats_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                RadioConfig::MAV_COMP_ID, &msg,
                                                &tm);
            break;
        }
        case SystemTMList::MAV_FSM_ID:
        {
            mavlink_fsm_tm_t tm;

            tm.timestamp = TimestampTimer::getTimestamp();
            tm.abk_state = 0;
            tm.ada_state = 0;
            tm.dpl_state = 0;
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);
            tm.wes_state = 0;

            mavlink_msg_fsm_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_PIN_OBS_ID:
        {
            mavlink_pin_tm_t tm;

            tm.timestamp = TimestampTimer::getTimestamp();
            tm.last_change_timestamp =
                modules.get<PinHandler>()
                    ->getPinsData()[PinsList::NOSECONE_PIN]
                    .lastStateTimestamp; /*<  Last change timestamp of pin*/
            tm.pin_id =
                PinsList::NOSECONE_PIN; /*<  A member of the PinsList enum*/
            tm.changes_counter =
                modules.get<PinHandler>()
                    ->getPinsData()[PinsList::NOSECONE_PIN]
                    .changesCount; /*<  Number of changes of pin*/
            tm.current_state = modules.get<PinHandler>()
                                   ->getPinsData()[PinsList::NOSECONE_PIN]
                                   .lastState; /*<  Current state of pin*/

            mavlink_msg_pin_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        default:
        {
            // Send by default the nack message
            mavlink_nack_tm_t nack;

            nack.recv_msgid = msgId;
            nack.seq_ack    = seq;

            LOG_ERR(logger, "Unknown message id: {}", tmId);
            mavlink_msg_nack_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                       RadioConfig::MAV_COMP_ID, &msg, &nack);

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

    switch (sensorId)
    {
        case SensorsTMList::MAV_GPS_ID:
        {
            mavlink_gps_tm_t tm;

            UBXGPSData gpsData = modules.get<Sensors>()->getUbxGpsLastSample();

            tm.timestamp = gpsData.gpsTimestamp;
            strcpy(tm.sensor_name, "UBXGPS");
            tm.fix          = gpsData.fix;
            tm.height       = gpsData.height;
            tm.latitude     = gpsData.latitude;
            tm.longitude    = gpsData.longitude;
            tm.n_satellites = gpsData.satellites;
            tm.speed        = gpsData.speed;
            tm.track        = gpsData.track;
            tm.vel_down     = gpsData.velocityDown;
            tm.vel_east     = gpsData.velocityEast;
            tm.vel_north    = gpsData.velocityNorth;

            mavlink_msg_gps_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_BMX160_ID:
        {
            mavlink_imu_tm_t tm;

            BMX160WithCorrectionData imuData =
                modules.get<Sensors>()->getBMX160WithCorrectionLastSample();

            strcpy(tm.sensor_name, "BMX160");
            tm.acc_x     = imuData.accelerationX;
            tm.acc_y     = imuData.accelerationY;
            tm.acc_z     = imuData.accelerationZ;
            tm.gyro_x    = imuData.angularSpeedX;
            tm.gyro_y    = imuData.angularSpeedY;
            tm.gyro_z    = imuData.angularSpeedZ;
            tm.mag_x     = imuData.magneticFieldX;
            tm.mag_y     = imuData.magneticFieldY;
            tm.mag_z     = imuData.magneticFieldZ;
            tm.timestamp = imuData.accelerationTimestamp;

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_MS5803_ID:
        {
            mavlink_pressure_tm_t tm;

            auto pressureData = modules.get<Sensors>()->getLPS22LastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS22");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_voltage_tm_t tm;
            BatteryVoltageSensorData voltage =
                modules.get<Sensors>()->getBatteryVoltageLastSample();
            tm.voltage = voltage.batVoltage;
            strcpy(tm.sensor_name, "BATTERY");
            tm.timestamp = voltage.voltageTimestamp;
            mavlink_msg_voltage_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                          RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_ADS_ID:
        {
            mavlink_adc_tm_t tm;
            ADS131M08Data voltage =
                modules.get<Sensors>()->getADS131LastSample();
            tm.channel_0 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_0).voltage;

            tm.channel_1 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_1).voltage;

            tm.channel_2 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_2).voltage;

            tm.channel_3 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_3).voltage;

            tm.channel_4 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_4).voltage;

            tm.channel_5 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_5).voltage;

            tm.channel_6 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_6).voltage;

            tm.channel_7 =
                voltage.getVoltage(ADS131M08Defs::Channel::CHANNEL_7).voltage;

            strcpy(tm.sensor_name, "ADS131M08");
            tm.timestamp = voltage.timestamp;
            mavlink_msg_adc_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        default:
        {
            mavlink_nack_tm_t nack;

            nack.recv_msgid = msgId;
            nack.seq_ack    = seq;

            LOG_DEBUG(logger, "Unknown telemetry id: {}", sensorId);
            mavlink_msg_nack_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                       RadioConfig::MAV_COMP_ID, &msg, &nack);
            break;
        }
    }

    return msg;
}

}  // namespace MockupMain
