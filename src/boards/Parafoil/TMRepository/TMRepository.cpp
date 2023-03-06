/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "TMRepository.h"

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/PinObserver/PinObserver.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil::SensorsConfig;

namespace Parafoil
{

mavlink_message_t TMRepository::packSystemTm(SystemTMList tmId, uint8_t msgId,
                                             uint8_t seq)
{
    ModuleManager& modules = ModuleManager::getInstance();
    mavlink_message_t msg;

    // Prevent preemption, MUST not yeld or use the kernel!
    PauseKernelLock kLock;

    switch (tmId)
    {
        case SystemTMList::MAV_SYS_ID:
        {
            mavlink_sys_tm_t tm;

            tm.timestamp    = TimestampTimer::getTimestamp();
            tm.logger       = Logger::getInstance().isStarted();
            tm.event_broker = EventBroker::getInstance().isRunning();
            tm.radio        = modules.get<Radio>()->isStarted();
            tm.pin_observer = PinObserver::getInstance().isRunning();
            tm.sensors      = modules.get<Sensors>()->isStarted();
            tm.board_scheduler =
                BoardScheduler::getInstance().getScheduler().isRunning();

            mavlink_msg_sys_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_LOGGER_ID:
        {
            mavlink_logger_tm_t tm;

            auto stats = Logger::getInstance().getStats();

            tm.timestamp          = stats.timestamp;
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

            mavlink_msg_logger_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                         RadioConfig::MAV_COMPONENT_ID, &msg,
                                         &tm);
            break;
        }
        case SystemTMList::MAV_MAVLINK_STATS:
        {
            mavlink_mavlink_stats_tm_t tm;

            auto stats = modules.get<Radio>()->getMavlinkStatus();

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
                                                RadioConfig::MAV_COMPONENT_ID,
                                                &msg, &tm);
            break;
        }
        case SystemTMList::MAV_NAS_ID:
        {
            mavlink_nas_tm_t tm;

            auto state = modules.get<NASController>()->getNasState();
            auto ref   = modules.get<NASController>()->getReferenceValues();

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
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            mavlink_payload_flight_tm_t tm;
            Sensors& sensors = *(modules.get<Sensors>());

            MS5803Data ms5803Data = sensors.getMS5803LastSample();
            BMX160WithCorrectionData imuData =
                sensors.getBMX160WithCorrectionLastSample();

            NASState nasState  = modules.get<NASController>()->getNasState();
            UBXGPSData ubxData = sensors.getUbxGpsLastSample();

            tm.timestamp = TimestampTimer::getTimestamp();

            // State machines states
            tm.ada_state = 0;
            tm.fmm_state =
                (uint8_t)modules.get<FlightModeManager>()->getStatus().state;
            tm.nas_state =
                (uint8_t)modules.get<NASController>()->getStatus().state;

            // Pressures
            tm.pressure_ada    = 0;
            tm.pressure_digi   = ms5803Data.pressure;
            tm.pressure_static = sensors.getStaticPressureLastSample().pressure;
            tm.pressure_dpl    = sensors.getDplPressureLastSample().pressure;
            tm.airspeed_pitot  = sensors.getPitotLastSample().airspeed;

            // Altitude agl
            tm.altitude_agl = -nasState.d;

            // IMU
            tm.acc_x  = imuData.accelerationX;
            tm.acc_y  = imuData.accelerationY;
            tm.acc_z  = imuData.accelerationZ;
            tm.gyro_x = imuData.angularSpeedX;
            tm.gyro_y = imuData.angularSpeedY;
            tm.gyro_z = imuData.angularSpeedZ;
            tm.mag_x  = imuData.magneticFieldX;
            tm.mag_y  = imuData.magneticFieldY;
            tm.mag_z  = imuData.magneticFieldZ;

            // GPS
            tm.gps_fix = ubxData.fix;
            tm.gps_lat = ubxData.latitude;
            tm.gps_lon = ubxData.longitude;
            tm.gps_alt = ubxData.height;

            // Servo motors
            tm.left_servo_angle =
                modules.get<Actuators>()->getServoAngle(PARAFOIL_LEFT_SERVO);
            tm.right_servo_angle =
                modules.get<Actuators>()->getServoAngle(PARAFOIL_RIGHT_SERVO);

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

            // Sensing pins statuses
            tm.pin_nosecone = modules.get<PinHandler>()
                                  ->getPinsData()[NOSECONE_PIN]
                                  .lastState;

            // Servo positions
            tm.left_servo_angle =
                modules.get<Actuators>()->getServoAngle(PARAFOIL_LEFT_SERVO);

            tm.right_servo_angle =
                modules.get<Actuators>()->getServoAngle(PARAFOIL_RIGHT_SERVO);

            // Board status
            tm.vbat         = sensors.getBatteryVoltageLastSample().batVoltage;
            tm.temperature  = ms5803Data.temperature;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;

            mavlink_msg_payload_flight_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                 RadioConfig::MAV_COMPONENT_ID,
                                                 &msg, &tm);
            break;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_payload_stats_tm_t tm;

            tm.cpu_load  = CpuMeter::getCpuStats().mean;
            tm.free_heap = CpuMeter::getCpuStats().freeHeap;

            mavlink_msg_payload_stats_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                RadioConfig::MAV_COMPONENT_ID,
                                                &msg, &tm);
            break;
        }
        case SystemTMList::MAV_FSM_ID:
        {
            mavlink_fsm_tm_t tm;

            tm.timestamp = TimestampTimer::getTimestamp();
            tm.abk_state = 0;
            tm.ada_state = 0;
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);

            mavlink_msg_fsm_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        default:
        {
            mavlink_nack_tm_t nack;

            nack.recv_msgid = msgId;
            nack.seq_ack    = seq;

            LOG_DEBUG(logger, "Unknown telemetry id: {}", tmId);
            mavlink_msg_nack_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                       RadioConfig::MAV_COMPONENT_ID, &msg,
                                       &nack);
            break;
        }
    }

    return msg;
}

mavlink_message_t TMRepository::packSensorsTm(SensorsTMList sensorId,
                                              uint8_t msgId, uint8_t seq)
{
    ModuleManager& modules = ModuleManager::getInstance();
    mavlink_message_t msg;

    switch (sensorId)
    {
        case SensorsTMList::MAV_GPS_ID:
        {
            mavlink_gps_tm_t tm;

            UBXGPSData gpsData = modules.get<Sensors>()->getUbxGpsLastSample();

            tm.timestamp = gpsData.gpsTimestamp;
            strcpy(tm.sensor_id, "UBXGPS");
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
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_BMX160_ID:
        {
            mavlink_imu_tm_t tm;

            auto imuData =
                modules.get<Sensors>()->getBMX160WithCorrectionLastSample();

            tm.timestamp = imuData.accelerationTimestamp;
            strcpy(tm.sensor_id, "BMX160");
            tm.acc_x  = imuData.accelerationX;
            tm.acc_y  = imuData.accelerationY;
            tm.acc_z  = imuData.accelerationZ;
            tm.gyro_x = imuData.angularSpeedX;
            tm.gyro_y = imuData.angularSpeedY;
            tm.gyro_z = imuData.angularSpeedZ;
            tm.mag_x  = imuData.magneticFieldX;
            tm.mag_y  = imuData.magneticFieldY;
            tm.mag_z  = imuData.magneticFieldZ;

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_MS5803_ID:
        {
            mavlink_pressure_tm_t tm;

            auto pressureData = modules.get<Sensors>()->getMS5803LastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_id, "MS5803");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMPONENT_ID, &msg,
                                           &tm);

            break;
        }
        case SensorsTMList::MAV_DPL_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            auto pressureData =
                modules.get<Sensors>()->getDplPressureLastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_id, "DPL_PRESSURE");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMPONENT_ID, &msg,
                                           &tm);

            break;
        }
        case SensorsTMList::MAV_STATIC_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            auto pressureData =
                modules.get<Sensors>()->getStaticPressureLastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_id, "STATIC_PRESSURE");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMPONENT_ID, &msg,
                                           &tm);

            break;
        }
        case SensorsTMList::MAV_PITOT_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            SSCDRRN015PDAData pitot =
                modules.get<Sensors>()->getPitotPressureLastSample();

            tm.timestamp = pitot.pressureTimestamp;
            tm.pressure  = pitot.pressure;
            strcpy(tm.sensor_id, "PITOT");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMPONENT_ID, &msg,
                                           &tm);
            break;
        }
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_adc_tm_t tm;

            BatteryVoltageSensorData battery =
                modules.get<Sensors>()->getBatteryVoltageLastSample();

            tm.timestamp = battery.voltageTimestamp;
            tm.channel_0 = battery.batVoltage;
            tm.channel_1 = 0;
            tm.channel_2 = 0;
            tm.channel_3 = 0;

            strcpy(tm.sensor_id, "BATTERY_VOLTAGE");

            mavlink_msg_adc_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);
            break;
        }
        default:
        {
            mavlink_nack_tm_t nack;

            nack.recv_msgid = msgId;
            nack.seq_ack    = seq;

            LOG_DEBUG(logger, "Unknown telemetry id: {}", sensorId);
            mavlink_msg_nack_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                       RadioConfig::MAV_COMPONENT_ID, &msg,
                                       &nack);
            break;
        }
    }

    return msg;
}

mavlink_message_t TMRepository::packServoTm(ServosList servoId, uint8_t msgId,
                                            uint8_t seq)
{
    mavlink_message_t msg;

    if (servoId == PARAFOIL_LEFT_SERVO || servoId == PARAFOIL_RIGHT_SERVO)
    {
        Actuators& actuators = *ModuleManager::getInstance().get<Actuators>();
        mavlink_servo_tm_t tm;

        tm.servo_id       = servoId;
        tm.servo_position = actuators.getServoAngle(servoId);

        mavlink_msg_servo_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                    RadioConfig::MAV_COMPONENT_ID, &msg, &tm);
    }
    else
    {
        mavlink_nack_tm_t nack;

        nack.recv_msgid = msgId;
        nack.seq_ack    = seq;

        mavlink_msg_nack_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                   RadioConfig::MAV_COMPONENT_ID, &msg, &nack);
    }

    return msg;
}

}  // namespace Parafoil
