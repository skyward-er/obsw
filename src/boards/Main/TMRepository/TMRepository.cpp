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
            tm.pin_expulsion = static_cast<uint8_t>(
                miosix::gpios::exp_sense::getPin().value());

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
                static_cast<uint8_t>(miosix::gpios::cut_sense::value());

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
            mavlink_rocket_stats_tm_t tm;

            // TODO
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
            tm.main_valve_state = modules.get<Actuators>()->getServoPosition(
                ServosList::MAIN_VALVE);
            // TODO add the venting valve to telemetry
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
    mavlink_message_t msg;

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