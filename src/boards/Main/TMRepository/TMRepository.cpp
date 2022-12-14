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

#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/Configs/SensorsConfig.h>
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/AirBrakesController/AirBrakesController.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <algorithms/NAS/StateInitializer.h>
#include <common/ReferenceConfig.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <utils/PinObserver/PinObserver.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main::SensorsConfig;

namespace Main
{

mavlink_message_t TMRepository::packSystemTm(SystemTMList tmId, uint8_t msgId,
                                             uint8_t seq)
{
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
            tm.radio        = Radio::getInstance().isStarted();
            tm.pin_observer = PinObserver::getInstance().isRunning();
            tm.sensors      = Sensors::getInstance().isStarted();
            tm.board_scheduler =
                BoardScheduler::getInstance().getScheduler().isRunning();

            mavlink_msg_sys_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_FSM_ID:
        {
            mavlink_fsm_tm_t tm;

            tm.timestamp = TimestampTimer::getTimestamp();
            tm.ada_state = static_cast<uint8_t>(
                ADAController::getInstance().getStatus().state);
            tm.abk_state = static_cast<uint8_t>(
                AirBrakesController::getInstance().getStatus().state);
            tm.dpl_state = static_cast<uint8_t>(
                Deployment::getInstance().getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                FlightModeManager::getInstance().getStatus().state);
            tm.nas_state = static_cast<uint8_t>(
                NASController::getInstance().getStatus().state);

            mavlink_msg_fsm_tm_encode(RadioConfig::MAV_SYSTEM_ID,
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

            auto stats = Radio::getInstance().getMavlinkStatus();

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
        case SystemTMList::MAV_ADA_ID:
        {
            mavlink_ada_tm_t tm;

            ADAController &ada = ADAController::getInstance();
            auto status        = ada.getStatus();
            auto state         = ada.getAdaState();
            auto ref           = ada.getReferenceValues();

            tm.timestamp       = state.timestamp;
            tm.state           = static_cast<uint8_t>(status.state);
            tm.kalman_x0       = state.x0;
            tm.kalman_x1       = state.x1;
            tm.kalman_x2       = state.x2;
            tm.vertical_speed  = state.verticalSpeed;
            tm.ref_altitude    = ref.refAltitude;
            tm.ref_pressure    = ref.refPressure;
            tm.ref_temperature = ref.refTemperature - 273.15;
            tm.msl_altitude    = state.mslAltitude;
            tm.msl_pressure    = ref.mslPressure;
            tm.msl_temperature = ref.mslTemperature - 273.15;
            tm.dpl_altitude    = ada.getDeploymentAltitude();

            mavlink_msg_ada_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_NAS_ID:
        {
            mavlink_nas_tm_t tm;

            auto state  = NASController::getInstance().getNasState();
            auto status = NASController::getInstance().getStatus();
            auto ref    = NASController::getInstance().getReferenceValues();

            tm.timestamp       = state.timestamp;
            tm.state           = static_cast<uint8_t>(status.state);
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
            tm.ref_temperature = ref.refTemperature - 273.15;
            tm.ref_latitude    = ref.refLatitude;
            tm.ref_longitude   = ref.refLongitude;

            mavlink_msg_nas_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            mavlink_rocket_flight_tm_t tm;
            Sensors &sensors = Sensors::getInstance();

            auto &ada = ADAController::getInstance();
            auto &fmm = FlightModeManager::getInstance();
            auto &dpl = Deployment::getInstance();
            auto &abk = AirBrakesController::getInstance();
            auto &nas = NASController::getInstance();

            auto ms5803Data = sensors.getMS5803LastSample();
            auto imuData    = sensors.getBMX160WithCorrectionLastSample();

            auto nasState      = nas.getNasState();
            auto adaState      = ada.getAdaState();
            UBXGPSData ubxData = sensors.getUbxGpsLastSample();

            tm.timestamp = TimestampTimer::getTimestamp();

            // State machines states
            tm.ada_state = static_cast<uint8_t>(ada.getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(fmm.getStatus().state);
            tm.dpl_state = static_cast<uint8_t>(dpl.getStatus().state);
            tm.abk_state = static_cast<uint8_t>(abk.getStatus().state);
            tm.nas_state = static_cast<uint8_t>(nas.getStatus().state);

            // Pressures
            tm.pressure_ada    = adaState.x0;
            tm.pressure_digi   = ms5803Data.pressure;
            tm.pressure_static = sensors.getStaticPressureLastSample().pressure;
            tm.pressure_dpl    = sensors.getDplPressureLastSample().pressure;
            tm.airspeed_pitot  = sensors.getPitotLastSample().airspeed;

            // ADA estimation
            tm.altitude_agl   = adaState.aglAltitude;
            tm.ada_vert_speed = adaState.verticalSpeed;

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

            // Airbrakes
            tm.abk_angle =
                Actuators::getInstance().getServoAngle(AIR_BRAKES_SERVO);
            tm.abk_estimated_cd = 0;

            // Load cell
            tm.parachute_load = sensors.getLoadCellLastSample().load;

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
            tm.pin_launch =
                PinHandler::getInstance().getPinsData()[LAUNCH_PIN].lastState;
            tm.pin_nosecone =
                PinHandler::getInstance().getPinsData()[NOSECONE_PIN].lastState;
            tm.pin_expulsion = PinHandler::getInstance()
                                   .getPinsData()[DEPLOYMENT_PIN]
                                   .lastState;

            // Cutter presence
            tm.cutter_presence = sensors.isCutterPresent();

            // Board status
            tm.vbat         = sensors.getBatteryVoltageLastSample().batVoltage;
            tm.temperature  = ms5803Data.temperature;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;

            mavlink_msg_rocket_flight_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                RadioConfig::MAV_COMPONENT_ID,
                                                &msg, &tm);
            break;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_rocket_stats_tm_t tm;

            tm           = FlightStatsRecorder::getInstance().getStats();
            tm.cpu_load  = CpuMeter::getCpuStats().mean;
            tm.free_heap = CpuMeter::getCpuStats().freeHeap;

            mavlink_msg_rocket_stats_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                               RadioConfig::MAV_COMPONENT_ID,
                                               &msg, &tm);
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
    mavlink_message_t msg;

    switch (sensorId)
    {
        case SensorsTMList::MAV_GPS_ID:
        {
            mavlink_gps_tm_t tm;

            UBXGPSData gpsData = Sensors::getInstance().getUbxGpsLastSample();

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
                Sensors::getInstance().getBMX160WithCorrectionLastSample();

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
        case SensorsTMList::MAV_VN100_ID:
        {
            mavlink_imu_tm_t tm;

            auto imuData = Sensors::getInstance().getVN100LastSample();

            tm.timestamp = imuData.accelerationTimestamp;
            strcpy(tm.sensor_id, "VN100");
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
        case SensorsTMList::MAV_MPU9250_ID:
        {
            mavlink_imu_tm_t tm;

            auto imuData = Sensors::getInstance().getMPU9250LastSample();

            tm.timestamp = imuData.accelerationTimestamp;
            strcpy(tm.sensor_id, "MPU9250");
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
        case SensorsTMList::MAV_ADS_ID:
        {
            mavlink_adc_tm_t tm;

            auto adcData = Sensors::getInstance().getADS131M04LastSample();

            tm.timestamp = adcData.timestamp;
            strcpy(tm.sensor_id, "ADS131M04");
            tm.channel_0 = adcData.voltage[0];
            tm.channel_1 = adcData.voltage[1];
            tm.channel_2 = adcData.voltage[2];
            tm.channel_3 = adcData.voltage[3];

            mavlink_msg_adc_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMPONENT_ID, &msg, &tm);

            break;
        }
        case SensorsTMList::MAV_MS5803_ID:
        {
            mavlink_pressure_tm_t tm;

            auto pressureData = Sensors::getInstance().getMS5803LastSample();

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
                Sensors::getInstance().getDplPressureLastSample();

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
                Sensors::getInstance().getStaticPressureLastSample();

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

            auto pressureData = Sensors::getInstance().getPitotLastSample();

            tm.timestamp = pressureData.timestamp;
            strcpy(tm.sensor_id, "PITOT");
            tm.pressure = pressureData.deltaP;

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMPONENT_ID, &msg,
                                           &tm);

            break;
        }
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_voltage_tm_t tm;

            auto voltageData =
                Sensors::getInstance().getBatteryVoltageLastSample();

            tm.timestamp = voltageData.voltageTimestamp;
            strcpy(tm.sensor_id, "BATTERY_VOLTAGE");
            tm.voltage = voltageData.voltage;

            mavlink_msg_voltage_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                          RadioConfig::MAV_COMPONENT_ID, &msg,
                                          &tm);

            break;
        }
        case SensorsTMList::MAV_LOAD_CELL_ID:
        {
            mavlink_load_tm_t tm;

            auto loadData = Sensors::getInstance().getLoadCellLastSample();

            tm.timestamp = loadData.loadTimestamp;
            strcpy(tm.sensor_id, "LOAD_CELL");
            tm.load = loadData.load;

            mavlink_msg_load_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                       RadioConfig::MAV_COMPONENT_ID, &msg,
                                       &tm);

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

    if (servoId == AIR_BRAKES_SERVO || servoId == EXPULSION_SERVO)
    {
        mavlink_servo_tm_t tm;

        tm.servo_id       = servoId;
        tm.servo_position = Actuators::getInstance().getServoAngle(servoId);

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

}  // namespace Main
