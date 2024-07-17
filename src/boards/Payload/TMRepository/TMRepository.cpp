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
#include <Payload/Actuators/Actuators.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Configs/RadioConfig.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/TMRepository/TMRepository.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace Boardcore;

namespace Payload
{
mavlink_message_t TMRepository::packSystemTm(SystemTMList tmId, uint8_t msgId,
                                             uint8_t seq)
{
    mavlink_message_t msg;

    switch (tmId)
    {
        case SystemTMList::MAV_SYS_ID:
        {
            mavlink_sys_tm_t tm;

            tm.timestamp       = TimestampTimer::getTimestamp();
            tm.logger          = Logger::getInstance().isStarted();
            tm.board_scheduler = getModule<BoardScheduler>()->isStarted();
            tm.event_broker    = EventBroker::getInstance().isRunning();
            tm.radio           = getModule<Radio>()->isStarted();
            tm.sensors         = getModule<Sensors>()->isStarted();
            tm.pin_observer    = getModule<PinHandler>()->isStarted();

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
            MavlinkStatus stats = getModule<Radio>()->mavDriver->getStatus();

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
            NASState state = getModule<NASController>()->getNasState();
            ReferenceValues ref =
                getModule<NASController>()->getReferenceValues();

            tm.timestamp = state.timestamp;
            tm.state     = static_cast<uint8_t>(
                getModule<NASController>()->getStatus().state);
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
            LPS28DFWData lps28dfw =
                getModule<Sensors>()->getLPS28DFWLastSample();
            IMUData imu    = getModule<Sensors>()->getIMULastSample();
            UBXGPSData gps = getModule<Sensors>()->getUBXGPSLastSample();
            Eigen::Vector2f wind =
                getModule<WindEstimation>()->getWindEstimationScheme();

            // NAS state
            NASState nasState = getModule<NASController>()->getNasState();

            tm.nas_state = static_cast<uint8_t>(
                getModule<NASController>()->getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                getModule<FlightModeManager>()->getStatus().state);
            tm.wes_state = static_cast<uint8_t>(
                getModule<WingController>()->getStatus().state);

            tm.wes_n = wind[0];
            tm.wes_e = wind[1];

            tm.pressure_digi = lps28dfw.pressure;
            tm.pressure_static =
                getModule<Sensors>()->getStaticPressureLastSample().pressure;
            // Pitot
            tm.airspeed_pitot =
                getModule<Sensors>()->getPitotLastSample().airspeed;

            // Altitude agl
            tm.altitude_agl = -nasState.d;

            // IMU
            tm.acc_x  = imu.accData.accelerationX;
            tm.acc_y  = imu.accData.accelerationY;
            tm.acc_z  = imu.accData.accelerationZ;
            tm.gyro_x = imu.gyroData.angularSpeedX;
            tm.gyro_y = imu.gyroData.angularSpeedY;
            tm.gyro_z = imu.gyroData.angularSpeedZ;

            // Magnetometer
            tm.mag_x = imu.magData.magneticFieldX;
            tm.mag_y = imu.magData.magneticFieldY;
            tm.mag_z = imu.magData.magneticFieldZ;

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
            tm.left_servo_angle = getModule<Actuators>()->getServoAngle(
                ServosList::PARAFOIL_LEFT_SERVO);
            tm.right_servo_angle = getModule<Actuators>()->getServoAngle(
                ServosList::PARAFOIL_RIGHT_SERVO);

            tm.pin_nosecone = getModule<PinHandler>()
                                  ->getPinData()[PinsList::NOSECONE_PIN]
                                  .lastState;

            tm.cutter_presence =
                static_cast<uint8_t>(adcs::cutterSense::value());

            tm.battery_voltage =
                getModule<Sensors>()->getBatteryVoltageLastSample().batVoltage;
            tm.current_consumption = 0;
            tm.temperature         = lps28dfw.temperature;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;

            tm.cam_battery_voltage = getModule<Sensors>()
                                         ->getCamBatteryVoltageLastSample()
                                         .batVoltage;

            mavlink_msg_payload_flight_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                 RadioConfig::MAV_COMP_ID, &msg,
                                                 &tm);

            break;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_payload_stats_tm_t tm =
                getModule<FlightStatsRecorder>()->getStats();
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
            tm.dpl_state = static_cast<uint8_t>(
                getModule<WingController>()->getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                getModule<FlightModeManager>()->getStatus().state);
            tm.nas_state = static_cast<uint8_t>(
                getModule<NASController>()->getStatus().state);
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
                getModule<PinHandler>()
                    ->getPinData()[PinsList::NOSECONE_PIN]
                    .lastStateTimestamp; /*<  Last change timestamp of pin*/
            tm.pin_id =
                PinsList::NOSECONE_PIN; /*<  A member of the PinsList enum*/
            tm.changes_counter =
                getModule<PinHandler>()
                    ->getPinData()[PinsList::NOSECONE_PIN]
                    .changesCount; /*<  Number of changes of pin*/
            tm.current_state = getModule<PinHandler>()
                                   ->getPinData()[PinsList::NOSECONE_PIN]
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

    switch (sensorId)
    {
        case SensorsTMList::MAV_GPS_ID:
        {
            mavlink_gps_tm_t tm;

            UBXGPSData gpsData = getModule<Sensors>()->getUBXGPSLastSample();

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

        case SensorsTMList::MAV_STATIC_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            HSCMRNN015PAData pressureData =
                getModule<Sensors>()->getStaticPressureLastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_name, "STATIC_PRESSURE");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_DPL_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            SSCMRNN030PAData pressureData =
                getModule<Sensors>()->getDynamicPressureLastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_name, "DYNAMIC_PRESSURE");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_PITOT_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            SSCMRNN030PAData pitot =
                getModule<Sensors>()->getDynamicPressureLastSample();

            tm.timestamp = pitot.pressureTimestamp;
            tm.pressure  = pitot.pressure;
            strcpy(tm.sensor_name, "PITOT_PRESSURE");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_LIS2MDL_ID:
        {
            mavlink_imu_tm_t tm;

            LIS2MDLData mag = getModule<Sensors>()->getLIS2MDLLastSample();

            tm.acc_x     = 0;
            tm.acc_y     = 0;
            tm.acc_z     = 0;
            tm.gyro_x    = 0;
            tm.gyro_y    = 0;
            tm.gyro_z    = 0;
            tm.mag_x     = mag.magneticFieldX;
            tm.mag_y     = mag.magneticFieldY;
            tm.mag_z     = mag.magneticFieldZ;
            tm.timestamp = mag.magneticFieldTimestamp;
            strcpy(tm.sensor_name, "LIS2MD");

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_LSM6DSRX_ID:
        {
            mavlink_imu_tm_t tm;

            LSM6DSRXData imu = getModule<Sensors>()->getLSM6DSRXLastSample();

            tm.acc_x  = imu.accelerationX;
            tm.acc_y  = imu.accelerationY;
            tm.acc_z  = imu.accelerationZ;
            tm.gyro_x = imu.angularSpeedX;
            tm.gyro_y = imu.angularSpeedY;
            tm.gyro_z = imu.angularSpeedZ;
            tm.mag_x  = 0;
            tm.mag_y  = 0;
            tm.mag_z  = 0;

            tm.timestamp = imu.accelerationTimestamp;
            strcpy(tm.sensor_name, "LSM6DSRX");

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_H3LIS331DL_ID:
        {
            mavlink_imu_tm_t tm;

            H3LIS331DLData imu =
                getModule<Sensors>()->getH3LIS331DLLastSample();

            tm.acc_x  = imu.accelerationX;
            tm.acc_y  = imu.accelerationY;
            tm.acc_z  = imu.accelerationZ;
            tm.gyro_x = 0;
            tm.gyro_y = 0;
            tm.gyro_z = 0;
            tm.mag_x  = 0;
            tm.mag_y  = 0;
            tm.mag_z  = 0;

            tm.timestamp = imu.accelerationTimestamp;
            strcpy(tm.sensor_name, "H3LIS331DL");

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_LPS22DF_ID:
        {
            mavlink_pressure_tm_t tm;

            LPS22DFData pressure = getModule<Sensors>()->getLPS22DFLastSample();

            tm.timestamp = pressure.pressureTimestamp;
            tm.pressure  = pressure.pressure;
            strcpy(tm.sensor_name, "LPS22DF");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_LPS28DFW_ID:
        {
            mavlink_pressure_tm_t tm;
            LPS28DFWData pressure =
                getModule<Sensors>()->getLPS28DFWLastSample();

            tm.timestamp = pressure.pressureTimestamp;
            tm.pressure  = pressure.pressure;
            strcpy(tm.sensor_name, "LPS28DFW");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_voltage_tm_t tm;
            BatteryVoltageSensorData voltage =
                getModule<Sensors>()->getBatteryVoltageLastSample();
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
                getModule<Sensors>()->getADS131M08LastSample();
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

mavlink_message_t TMRepository::packServoTm(ServosList servoId, uint8_t msgId,
                                            uint8_t seq)
{
    mavlink_message_t msg;

    if (servoId == PARAFOIL_LEFT_SERVO || servoId == PARAFOIL_RIGHT_SERVO)
    {
        mavlink_servo_tm_t tm;

        tm.servo_id       = servoId;
        tm.servo_position = getModule<Actuators>()->getServoAngle(servoId);

        mavlink_msg_servo_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                    RadioConfig::MAV_COMP_ID, &msg, &tm);
    }
    else
    {
        mavlink_nack_tm_t nack;

        nack.recv_msgid = msgId;
        nack.seq_ack    = seq;

        mavlink_msg_nack_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                   RadioConfig::MAV_COMP_ID, &msg, &nack);
    }

    return msg;
}

}  // namespace Payload
