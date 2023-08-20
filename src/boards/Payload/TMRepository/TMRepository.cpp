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
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/TMRepository/TMRepository.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;

namespace Payload
{
mavlink_message_t TMRepository::packSystemTm(SystemTMList tmId, uint8_t msgId,
                                             uint8_t seq)
{
    ModuleManager& modules = ModuleManager::getInstance();
    mavlink_message_t msg;

    // Prevent context switch
    PauseKernelLock lock;

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
            // TODO pin observer
            tm.pin_observer = 0;

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
            LPS28DFWData lps28dfw1 =
                modules.get<Sensors>()->getLPS28DFW_1LastSample();
            // LPS28DFWData lps28dfw2 =
            //     modules.get<Sensors>()->getLPS28DFW_2LastSample();
            LIS2MDLData lis2mdl =
                modules.get<Sensors>()->getLIS2MDLLastSample();
            UBXGPSData gps = modules.get<Sensors>()->getGPSLastSample();
            LSM6DSRXData lsm6dsrx =
                modules.get<Sensors>()->getLSM6DSRXLastSample();

            // NAS state
            NASState nasState = modules.get<NASController>()->getNasState();

            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            // TODO add wing state and estimation
            tm.wes_state = 0;
            tm.wes_e     = 0;
            tm.wes_n     = 0;

            tm.pressure_digi = lps28dfw1.pressure;
            tm.pressure_static =
                modules.get<Sensors>()->getStaticPressureLastSample().pressure;
            // Pitot
            tm.airspeed_pitot =
                modules.get<Sensors>()->getPitotLastSample().airspeed;

            // Altitude agl
            tm.altitude_agl = -nasState.d;

            // IMU
            tm.acc_x  = lsm6dsrx.accelerationX;
            tm.acc_y  = lsm6dsrx.accelerationY;
            tm.acc_z  = lsm6dsrx.accelerationZ;
            tm.gyro_x = lsm6dsrx.angularSpeedX;
            tm.gyro_y = lsm6dsrx.angularSpeedY;
            tm.gyro_z = lsm6dsrx.angularSpeedZ;

            // Magnetometer
            tm.mag_x = lis2mdl.magneticFieldX;
            tm.mag_y = lis2mdl.magneticFieldY;
            tm.mag_z = lis2mdl.magneticFieldZ;

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
            tm.left_servo_angle =
                ModuleManager::getInstance().get<Actuators>()->getServoAngle(
                    ServosList::PARAFOIL_LEFT_SERVO);
            tm.right_servo_angle =
                ModuleManager::getInstance().get<Actuators>()->getServoAngle(
                    ServosList::PARAFOIL_RIGHT_SERVO);
            ;

            // TODO Pins
            tm.pin_nosecone = 0;

            // TODO Board status
            tm.logger_error = 0;
            tm.temperature  = 0;
            // TODO Current and Voltage
            tm.battery_voltage     = 0;
            tm.cam_battery_voltage = 0;
            tm.current_consumption = 0;
            tm.vsupply_5v          = 0;

            mavlink_msg_payload_flight_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                 RadioConfig::MAV_COMP_ID, &msg,
                                                 &tm);

            break;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_rocket_stats_tm_t tm;
            // tm.liftoff_ts;         /*< [us] System clock at liftoff*/
            // tm.liftoff_max_acc_ts; /*< [us] System clock at the maximum
            // liftoff
            //                           acceleration*/
            // tm.dpl_ts;             /*< [us] System clock at drouge
            // deployment*/ tm.max_z_speed_ts; /*< [us] System clock at ADA max
            // vertical speed*/ tm.apogee_ts;      /*< [us] System clock at
            // apogee*/ tm.liftoff_max_acc;    /*< [m/s2] Maximum liftoff
            // acceleration*/ tm.dpl_max_acc;        /*< [m/s2] Max acceleration
            // during drouge
            //                           deployment*/
            // tm.max_z_speed;        /*< [m/s] Max measured vertical speed -
            // ADA*/ tm.max_airspeed_pitot; /*< [m/s] Max speed read by the
            // pitot tube*/ tm.max_speed_altitude; /*< [m] Altitude at max
            // speed*/ tm.apogee_lat;         /*< [deg] Apogee latitude*/
            // tm.apogee_lon;         /*< [deg] Apogee longitude*/
            // tm.apogee_alt;         /*< [m] Apogee altitude*/
            // tm.min_pressure;     /*< [Pa] Apogee pressure - Digital
            // barometer*/ tm.ada_min_pressure; /*< [Pa] Apogee pressure - ADA
            // filtered*/ tm.dpl_vane_max_pressure; /*< [Pa] Max pressure in the
            // deployment
            //                              bay during drogue deployment*/
            // tm.cpu_load;              /*<  CPU load in percentage*/
            // tm.free_heap;             /*<  Amount of available heap in
            // memory*/

            // TODO when implemented stats
            tm.cpu_load  = CpuMeter::getCpuStats().mean;
            tm.free_heap = CpuMeter::getCpuStats().freeHeap;

            mavlink_msg_rocket_stats_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                               RadioConfig::MAV_COMP_ID, &msg,
                                               &tm);
            break;
        }
        case SystemTMList::MAV_FSM_ID:
        {
            mavlink_fsm_tm_t tm;

            // TODO update when finished with FSM
            tm.timestamp = TimestampTimer::getTimestamp();
            tm.abk_state = 0;
            tm.ada_state = 0;
            tm.dpl_state = 0;
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            ;
            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);
            tm.wes_state = 0;

            mavlink_msg_fsm_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_TASK_STATS_ID:
        {
            break;
        }
        case SystemTMList::MAV_PIN_OBS_ID:
        {
            mavlink_pin_tm_t tm;

            // TODO update when PINobserver
            tm.timestamp = TimestampTimer::getTimestamp();
            // tm.last_change_timestamp; /*<  Last change timestamp of pin*/
            // tm.pin_id;                /*<  A member of the PinsList enum*/
            // tm.changes_counter;       /*<  Number of changes of pin*/
            // tm.current_state;         /*<  Current state of pin*/

            mavlink_msg_pin_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_SENSORS_STATE_ID:
        {
            // TODO with matteo when ready
            break;
        }
        case SystemTMList::MAV_CAN_ID:
        {
            // TODO Add to mavlinkLIB

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

            UBXGPSData gpsData = modules.get<Sensors>()->getGPSLastSample();

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
                modules.get<Sensors>()->getStaticPressureLastSample();

            tm.timestamp = pressureData.pressureTimestamp;
            strcpy(tm.sensor_name, "STATIC_PRESSURE");
            tm.pressure = pressureData.pressure;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_PITOT_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            SSCMRNN030PAData pitot =
                modules.get<Sensors>()->getDynamicPressureLastSample();

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

            LIS2MDLData mag = modules.get<Sensors>()->getLIS2MDLLastSample();

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

            LSM6DSRXData imu = modules.get<Sensors>()->getLSM6DSRXLastSample();

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
                modules.get<Sensors>()->getH3LIS331DLLastSample();

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

            LPS22DFData pressure =
                modules.get<Sensors>()->getLPS22DFLastSample();

            tm.timestamp = pressure.pressureTimestamp;
            tm.pressure  = pressure.pressure;
            strcpy(tm.sensor_name, "LPS22DF");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);
            break;
        }
        case SensorsTMList::MAV_CURRENT_SENSE_ID:
        {
            // TODO add current
        }
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
        {
            // TODO add battery voltage
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

        tm.servo_id = servoId;
        tm.servo_position =
            ModuleManager::getInstance().get<Actuators>()->getServoAngle(
                servoId);

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
