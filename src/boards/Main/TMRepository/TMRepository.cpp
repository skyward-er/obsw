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
#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/Configs/RadioConfig.h>
#include <Main/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Radio/Radio.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ABKController/ABKController.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/Deployment/Deployment.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/MEAController/MEAController.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <Main/TMRepository/TMRepository.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace Boardcore;

namespace Main
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
        case SystemTMList::MAV_ADA_ID:
        {
            mavlink_ada_tm_t tm;

            // Get the current ADA state
            ADAState state = modules.get<ADAController>()->getADAState();
            ReferenceValues ref =
                modules.get<ADAController>()->getReferenceValues();

            tm.timestamp = state.timestamp;
            tm.state     = static_cast<uint8_t>(
                modules.get<ADAController>()->getStatus().state);
            tm.kalman_x0       = state.x0;
            tm.kalman_x1       = state.x1;
            tm.kalman_x2       = state.x2;
            tm.vertical_speed  = state.verticalSpeed;
            tm.msl_altitude    = state.mslAltitude;
            tm.msl_pressure    = ref.mslPressure;
            tm.msl_temperature = ref.mslTemperature - 273.15f;
            tm.ref_altitude    = ref.refAltitude;
            tm.ref_temperature = ref.refTemperature - 273.15f;
            tm.ref_pressure    = ref.refPressure;
            tm.dpl_altitude    = 0;

            mavlink_msg_ada_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            mavlink_rocket_flight_tm_t tm;

            tm.timestamp = TimestampTimer::getTimestamp();

            // Last samples
            LSM6DSRXData lsm6dsrx =
                modules.get<Sensors>()->getLSM6DSRXLastSample();
            LPS28DFWData lps28dfw1 =
                modules.get<Sensors>()->getLPS28DFW_1LastSample();
            LIS2MDLData lis2mdl =
                modules.get<Sensors>()->getLIS2MDLLastSample();
            PressureData staticPressure =
                modules.get<Sensors>()->getStaticPressure1LastSample();
            PressureData deploymentPressure =
                modules.get<Sensors>()->getDeploymentPressureLastSample();
            UBXGPSData gps = modules.get<Sensors>()->getGPSLastSample();

            // NAS state
            NASState nasState = modules.get<NASController>()->getNasState();

            // ADA state
            ADAState adaState = modules.get<ADAController>()->getADAState();

            // State machines
            tm.mea_state = static_cast<uint8_t>(
                modules.get<MEAController>()->getStatus().state);
            tm.ada_state = static_cast<uint8_t>(
                modules.get<ADAController>()->getStatus().state);
            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);
            tm.abk_state = static_cast<uint8_t>(
                modules.get<ABKController>()->getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            tm.dpl_state = static_cast<uint8_t>(
                modules.get<Deployment>()->getStatus().state);

            // Pressures
            tm.pressure_ada    = adaState.x0;
            tm.pressure_digi   = lps28dfw1.pressure;
            tm.pressure_dpl    = deploymentPressure.pressure;
            tm.pressure_static = staticPressure.pressure;

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

            // ADA
            tm.ada_vert_speed = adaState.verticalSpeed;

            // ABK
            tm.abk_angle = modules.get<Actuators>()->getServoPosition(
                ServosList::AIR_BRAKES_SERVO);

            // Pins
            tm.pin_launch = modules.get<PinHandler>()
                                ->getPinData(PinHandler::PinList::LAUNCH_PIN)
                                .lastState;
            tm.pin_nosecone =
                modules.get<PinHandler>()
                    ->getPinData(PinHandler::PinList::NOSECONE_PIN)
                    .lastState;
            tm.pin_expulsion =
                modules.get<PinHandler>()
                    ->getPinData(PinHandler::PinList::PIN_EXPULSION)
                    .lastState;

            // Board status
            tm.battery_voltage = modules.get<Sensors>()
                                     ->getBatteryVoltageLastSample()
                                     .batVoltage;
            tm.cam_battery_voltage = modules.get<Sensors>()
                                         ->getCamBatteryVoltageLastSample()
                                         .batVoltage;
            tm.current_consumption =
                modules.get<Sensors>()->getCurrentLastSample().current;
            tm.temperature  = lps28dfw1.temperature;
            tm.logger_error = Logger::getInstance().getStats().lastWriteError;
            tm.cutter_presence =
                modules.get<PinHandler>()
                    ->getPinData(PinHandler::PinList::CUTTER_PRESENCE)
                    .lastState;

            // Pitot CAN sensor
            tm.airspeed_pitot =
                modules.get<Sensors>()->getPitotLastSample().airspeed;

            mavlink_msg_rocket_flight_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                                RadioConfig::MAV_COMP_ID, &msg,
                                                &tm);

            break;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_rocket_stats_tm_t tm =
                modules.get<FlightStatsRecorder>()->getStats();

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

            tm.timestamp = TimestampTimer::getTimestamp();
            tm.abk_state = static_cast<uint8_t>(
                modules.get<ABKController>()->getStatus().state);
            tm.ada_state = static_cast<uint8_t>(
                modules.get<ADAController>()->getStatus().state);
            tm.dpl_state = static_cast<uint8_t>(
                modules.get<Deployment>()->getStatus().state);
            tm.fmm_state = static_cast<uint8_t>(
                modules.get<FlightModeManager>()->getStatus().state);
            tm.nas_state = static_cast<uint8_t>(
                modules.get<NASController>()->getStatus().state);

            mavlink_msg_fsm_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case SystemTMList::MAV_MOTOR_ID:
        {
            mavlink_motor_tm_t tm;

            tm.timestamp            = TimestampTimer::getTimestamp();
            tm.bottom_tank_pressure = modules.get<Sensors>()
                                          ->getBottomTankPressureLastSample()
                                          .pressure;
            tm.combustion_chamber_pressure =
                modules.get<Sensors>()->getCCPressureLastSample().pressure;
            tm.tank_temperature = modules.get<Sensors>()
                                      ->getTankTemperatureLastSample()
                                      .temperature;
            tm.main_valve_state    = modules.get<Actuators>()->getServoPosition(
                                      ServosList::MAIN_VALVE) > 0.3
                                         ? 1
                                         : 0;
            tm.venting_valve_state = modules.get<Actuators>()->getServoPosition(
                                         ServosList::VENTING_VALVE) > 0.3
                                         ? 1
                                         : 0;
            tm.top_tank_pressure =
                modules.get<Sensors>()->getTopTankPressureLastSample().pressure;
            tm.battery_voltage =
                modules.get<Sensors>()->getMotorBatteryVoltage().batVoltage;
            tm.current_consumption =
                modules.get<Sensors>()->getMotorCurrent().current;

            mavlink_msg_motor_tm_encode(RadioConfig::MAV_SYSTEM_ID,
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
    ModuleManager& modules = ModuleManager::getInstance();
    mavlink_message_t msg;

    switch (sensorId)
    {
        case MAV_GPS_ID:
        {
            mavlink_gps_tm_t tm;

            UBXGPSData data = modules.get<Sensors>()->getGPSLastSample();

            tm.fix          = data.fix;
            tm.height       = data.height;
            tm.latitude     = data.latitude;
            tm.longitude    = data.longitude;
            tm.n_satellites = data.satellites;
            strcpy(tm.sensor_name, "UBXGPS");
            tm.speed     = data.speed;
            tm.timestamp = data.gpsTimestamp;
            tm.track     = data.track;
            tm.vel_down  = data.velocityDown;
            tm.vel_east  = data.velocityEast;
            tm.vel_north = data.velocityNorth;

            mavlink_msg_gps_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_ADS_ID:
        {
            mavlink_adc_tm_t tm;

            ADS131M08Data data =
                modules.get<Sensors>()->getADS131M08LastSample();

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

            mavlink_msg_adc_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_CURRENT_SENSE_ID:
        {
            mavlink_current_tm_t tm;

            CurrentData data = modules.get<Sensors>()->getCurrentLastSample();

            tm.current   = data.current;
            tm.timestamp = data.currentTimestamp;
            strcpy(tm.sensor_name, "CURRENT");

            mavlink_msg_current_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                          RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_DPL_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData data =
                modules.get<Sensors>()->getDeploymentPressureLastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "DPL_PRESSURE");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_STATIC_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData data =
                modules.get<Sensors>()->getStaticPressure1LastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "STATIC_PRESSURE");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_BATTERY_VOLTAGE_ID:
        {
            mavlink_voltage_tm_t tm;

            BatteryVoltageSensorData data =
                modules.get<Sensors>()->getBatteryVoltageLastSample();

            tm.voltage   = data.batVoltage;
            tm.timestamp = data.voltageTimestamp;
            strcpy(tm.sensor_name, "BATTERY_VOLTAGE");

            mavlink_msg_voltage_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                          RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_TANK_TOP_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData data =
                modules.get<Sensors>()->getTopTankPressureLastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "TOP_TANK");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_TANK_BOTTOM_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData data =
                modules.get<Sensors>()->getBottomTankPressureLastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "BOTTOM_TANK");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_COMBUSTION_PRESS_ID:
        {
            mavlink_pressure_tm_t tm;

            PressureData data =
                modules.get<Sensors>()->getCCPressureLastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "COMBUSTION_CHAMBER");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_TANK_TEMP_ID:
        {
            mavlink_temp_tm_t tm;

            TemperatureData data =
                modules.get<Sensors>()->getTankTemperatureLastSample();

            tm.temperature = data.temperature;
            tm.timestamp   = data.temperatureTimestamp;
            strcpy(tm.sensor_name, "TANK_TEMPERATURE");

            mavlink_msg_temp_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                       RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_LIS2MDL_ID:
        {
            mavlink_imu_tm_t tm;

            LIS2MDLData data = modules.get<Sensors>()->getLIS2MDLLastSample();

            tm.acc_x  = 0;
            tm.acc_y  = 0;
            tm.acc_z  = 0;
            tm.gyro_x = 0;
            tm.gyro_y = 0;
            tm.gyro_z = 0;

            tm.mag_x     = data.magneticFieldX;
            tm.mag_y     = data.magneticFieldY;
            tm.mag_z     = data.magneticFieldZ;
            tm.timestamp = data.magneticFieldTimestamp;
            strcpy(tm.sensor_name, "LIS2MDL");

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_LPS28DFW_ID:
        {
            mavlink_pressure_tm_t tm;

            LPS28DFWData data =
                modules.get<Sensors>()->getLPS28DFW_1LastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS28DFW");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_LSM6DSRX_ID:
        {
            mavlink_imu_tm_t tm;

            LSM6DSRXData data = modules.get<Sensors>()->getLSM6DSRXLastSample();

            tm.mag_x     = 0;
            tm.mag_y     = 0;
            tm.mag_z     = 0;
            tm.acc_x     = data.accelerationX;
            tm.acc_y     = data.accelerationY;
            tm.acc_z     = data.accelerationZ;
            tm.gyro_x    = data.angularSpeedX;
            tm.gyro_y    = data.angularSpeedY;
            tm.gyro_z    = data.angularSpeedZ;
            tm.timestamp = data.accelerationTimestamp;
            strcpy(tm.sensor_name, "LSM6DSRX");

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_H3LIS331DL_ID:
        {
            mavlink_imu_tm_t tm;

            H3LIS331DLData data =
                modules.get<Sensors>()->getH3LIS331DLLastSample();

            tm.mag_x     = 0;
            tm.mag_y     = 0;
            tm.mag_z     = 0;
            tm.gyro_x    = 0;
            tm.gyro_y    = 0;
            tm.gyro_z    = 0;
            tm.acc_x     = data.accelerationX;
            tm.acc_y     = data.accelerationY;
            tm.acc_z     = data.accelerationZ;
            tm.timestamp = data.accelerationTimestamp;
            strcpy(tm.sensor_name, "H3LIS331DL");

            mavlink_msg_imu_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                      RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        case MAV_LPS22DF_ID:
        {
            mavlink_pressure_tm_t tm;

            LPS22DFData data = modules.get<Sensors>()->getLPS22DFLastSample();

            tm.pressure  = data.pressure;
            tm.timestamp = data.pressureTimestamp;
            strcpy(tm.sensor_name, "LPS22DF");

            mavlink_msg_pressure_tm_encode(RadioConfig::MAV_SYSTEM_ID,
                                           RadioConfig::MAV_COMP_ID, &msg, &tm);

            break;
        }
        default:
        {
            // Send by default the nack message
            mavlink_nack_tm_t nack;

            nack.recv_msgid = msgId;
            nack.seq_ack    = seq;

            LOG_ERR(logger, "Unknown sensor id: {}", sensorId);
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
    ModuleManager& modules = ModuleManager::getInstance();
    mavlink_message_t msg;

    if (servoId == AIR_BRAKES_SERVO || servoId == EXPULSION_SERVO ||
        servoId == MAIN_VALVE || servoId == VENTING_VALVE)
    {
        mavlink_servo_tm_t tm;
        tm.servo_id       = servoId;
        tm.servo_position = modules.get<Actuators>()->getServoPosition(servoId);

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
}  // namespace Main