/* Copyright (c) 2015-2019 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron, Alessio Galluccio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "LogProxy.h"
#include "DeathStack/Status.h"

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack/DeploymentController/DeploymentData.h"
#include "DeathStack/FlightModeManager/FMMStatus.h"
#include "DeathStack/IgnitionController/IgnitionStatus.h"
#include "DeathStack/PinObserver/PinObserverData.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapperData.h"
#include "DeathStack/SensorManager/SensorManagerData.h"

#include "skyward-boardcore/src/shared/drivers/canbus/CanUtils.h"
#include "skyward-boardcore/src/shared/drivers/mavlink/MavStatus.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include "sensors/ADIS16405/ADIS16405Data.h"
#include "scheduler/TaskSchedulerData.h"
#include "drivers/piksi/piksi_data.h"

using namespace Status;

namespace DeathStackBoard
{

/**
 * Each function here is an implementation of the log() method for a
 * specific status struct or class.
 * The rationale is that, whenever a status struct is logged, the corresponding
 * telemetry struct is updated synchronously in the telemetry repo.
 * Telemetry requests from ground will therefore return the latest logged struct
 * from each component.
 */

/* Flight Mode Manager */
template <>
LogResult LoggerProxy::log<FMMStatus>(const FMMStatus& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.hm1_tm.state = static_cast<uint8_t>(t.state);
    tm_repository.lr_tm.fmm_state = static_cast<uint8_t>(t.state);

    return logger.log(t);
}

/* Launch and Nosecone detachment pins */
template <>
LogResult LoggerProxy::log<PinStatus>(const PinStatus& t)
{
	miosix::PauseKernelLock kLock;

    switch(t.pin)
    {
        /*
        case ObservedPin::LAUNCH:
            tm_repository.hm1_tm.launch_pin_last_detection = t.last_triggered_time;
            tm_repository.hm1_tm.launch_pin_detection_count = t.num_triggered;
            break;
        // TODO Nosecone pin in telemetria
        case ObservedPin::NOSECONE:
            tm_repository.hm1_tm.nc_pin_last_detection = t.last_triggered_time;
            tm_repository.hm1_tm.nc_pin_detection_count = t.num_triggered;
            break;
            */
        default:
            break;
    }

    return logger.log(t);
}

/* Ignition Board */
template <>
LogResult LoggerProxy::log<IgnBoardLoggableStatus>(const IgnBoardLoggableStatus& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.ign_tm.timestamp = t.timestamp;
    uint8_t* bitfield_ptr = (uint8_t*)(&(t.board_status));
    memcpy( &(tm_repository.ign_tm.avr_bitfield),
            &(bitfield_ptr[0]),
            sizeof(uint8_t) );
    memcpy( &(tm_repository.ign_tm.stm32_bitfield),
            &(bitfield_ptr[1]),
            sizeof(uint8_t) );                  // Bitfield

    return logger.log(t);
}

/* Logger */
template <>
LogResult LoggerProxy::log<LogStats>(const LogStats& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.logger_tm.timestamp = t.timestamp;
    tm_repository.logger_tm.statTooLargeSamples = t.statTooLargeSamples;
    tm_repository.logger_tm.statDroppedSamples = t.statDroppedSamples;
    tm_repository.logger_tm.statQueuedSamples = t.statQueuedSamples;
    tm_repository.logger_tm.statBufferFilled = t.statBufferFilled;
    tm_repository.logger_tm.statBufferWritten = t.statBufferWritten;
    tm_repository.logger_tm.statWriteFailed = t.statWriteFailed;
    tm_repository.logger_tm.statWriteError = t.statWriteError;
    tm_repository.logger_tm.statWriteTime = t.statWriteTime;
    tm_repository.logger_tm.statMaxWriteTime = t.statMaxWriteTime;

    return logger.log(t);
}

/* TMTCManager (Mavlink) */
template <>
LogResult LoggerProxy::log<MavStatus>(const MavStatus& t)
{
	miosix::PauseKernelLock kLock;

    // mavchannel stats
    tm_repository.tmtc_tm.timestamp = t.timestamp;
    tm_repository.tmtc_tm.n_send_queue = t.n_send_queue;
    tm_repository.tmtc_tm.max_send_queue = t.max_send_queue;
    tm_repository.tmtc_tm.n_send_errors = t.n_send_errors;
    tm_repository.tmtc_tm.msg_received = t.mav_stats.msg_received;
    // mav stats
    tm_repository.tmtc_tm.buffer_overrun = t.mav_stats.buffer_overrun;
    tm_repository.tmtc_tm.parse_error = t.mav_stats.parse_error;
    tm_repository.tmtc_tm.parse_state = t.mav_stats.parse_state;
    tm_repository.tmtc_tm.packet_idx = t.mav_stats.packet_idx;
    tm_repository.tmtc_tm.current_rx_seq = t.mav_stats.current_rx_seq;
    tm_repository.tmtc_tm.current_tx_seq = t.mav_stats.current_tx_seq;
    tm_repository.tmtc_tm.packet_rx_success_count = t.mav_stats.packet_rx_success_count;
    tm_repository.tmtc_tm.packet_rx_drop_count = t.mav_stats.packet_rx_drop_count;

    return logger.log(t);
}

/* Sensor Manager */
template <>
LogResult LoggerProxy::log<SensorManagerStatus>(const SensorManagerStatus& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.sm_tm.timestamp = t.timestamp;
    tm_repository.sm_tm.sensor_init_state = t.sensor_status;
    tm_repository.sm_tm.state = (uint8_t) t.state;

    return logger.log(t);
}

/* Ignition Controller */
template <>
LogResult LoggerProxy::log<IgnCtrlStatus>(const IgnCtrlStatus& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.ign_ctrl_tm.timestamp = t.timestamp;
    tm_repository.ign_ctrl_tm.fsm_state = t.fsm_state;
    tm_repository.ign_ctrl_tm.last_event = t.last_event;
    // TODO add s
    tm_repository.ign_ctrl_tm.n_rcv_message = t.n_rcv_messages;
    tm_repository.ign_ctrl_tm.n_sent_messages = t.n_sent_messages;

    // Bitfield
    tm_repository.ign_ctrl_tm.cmd_bitfield = 0;
    tm_repository.ign_ctrl_tm.cmd_bitfield &= t.launch_sent;
    tm_repository.ign_ctrl_tm.cmd_bitfield &= (t.abort_sent << 1);
    tm_repository.ign_ctrl_tm.cmd_bitfield &= (t.abort_rcv << 2);

    return logger.log(t);
}

/* Deployment Controller */
template <>
LogResult LoggerProxy::log<DeploymentStatus>(const DeploymentStatus& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.dpl_ctrl_tm.timestamp = t.timestamp;
    tm_repository.dpl_ctrl_tm.fsm_state = (uint8_t) t.state;
    tm_repository.dpl_ctrl_tm.motor_active = t.motor_status.motor_active;
    tm_repository.dpl_ctrl_tm.motor_last_direction = (uint8_t) t.motor_status.motor_last_direction;
    tm_repository.dpl_ctrl_tm.cutter_state = (uint8_t) t.cutter_status.state;

    return logger.log(t);
}

/* ADA state machine */
template <>
LogResult LoggerProxy::log<ADAStatus>(const ADAStatus& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.ada_tm.timestamp = t.timestamp;
    tm_repository.ada_tm.state = (uint8_t) t.state;
   
    return logger.log(t);
}

/* ADA apogee */
template <>
LogResult LoggerProxy::log<ApogeeDetected>(const ApogeeDetected& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.ada_tm.last_apogee_state = (uint8_t) t.state;
    tm_repository.ada_tm.last_apogee_tick = t.tick;
   
    return logger.log(t);
}

/* ADA DplPressure */
template <>
LogResult LoggerProxy::log<DplAltitudeReached>(const DplAltitudeReached& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.ada_tm.last_dpl_pressure_tick = t.tick;
   
    return logger.log(t);
}

/* ADA target dpl pressure */
template <>
LogResult LoggerProxy::log<TargetDeploymentAltitude>(const TargetDeploymentAltitude& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.ada_tm.target_dpl_pressure = t.deployment_altitude;
   
    return logger.log(t);
}

/* ADA kalman filter values */
template <>
LogResult LoggerProxy::log<KalmanState>(const KalmanState& t)
{
	miosix::PauseKernelLock kLock;

    tm_repository.ada_tm.kalman_x0 = t.x0;
    tm_repository.ada_tm.kalman_x1 = t.x1;
    tm_repository.ada_tm.kalman_x2 = t.x2;

    return logger.log(t);
}

/* Canbus stats */
template <>
LogResult LoggerProxy::log<CanStatus>(const CanStatus& t)
{


    tm_repository.can_tm.n_sent = t.n_sent;
    tm_repository.can_tm.n_rcv = t.n_rcv;
    tm_repository.can_tm.last_sent = t.last_sent;
    tm_repository.can_tm.last_rcv = t.last_rcv;
    tm_repository.can_tm.last_sent_ts = t.last_sent_ts;
    tm_repository.can_tm.last_rcv_ts = t.last_rcv_ts;

    return logger.log(t);
}

/* Main Barometer */
template <>
LogResult LoggerProxy::log<AD7994WrapperData>(const AD7994WrapperData& t)
{


    // TODO manca val_ch1_min
    tm_repository.ad7994_tm.timestamp = t.timestamp;
    tm_repository.ad7994_tm.val_ch1 = t.nxp_baro_pressure;
    
    //t.val_ch1_min
    //tm_repository.ad7994_tm.mean_ch1 = t.baro_1_mean;
    //tm_repository.ad7994_tm.stddev_ch1 = t.baro_1_stddev;
    tm_repository.ad7994_tm.val_ch2 = t.honeywell_baro_pressure;

    // LR_TM
    tm_repository.lr_tm.baro1_val = t.nxp_baro_pressure;
    //tm_repository.lr_tm.baro1_min = t.val_ch1_min;

    return logger.log(t);
}

/* Battery status, sampled by internal ADC */
template <>
LogResult LoggerProxy::log<BatteryVoltageData>(const BatteryVoltageData& t)
{


    tm_repository.adc_tm.timestamp = t.timestamp;
    tm_repository.adc_tm.battery_tension = t.battery_voltage_value;
    tm_repository.adc_tm.battery_tension_min = t.battery_voltage_min;
    // LR_TM
    tm_repository.lr_tm.battery_tension_min = t.battery_voltage_min;
   
    return logger.log(t);
}

/* Motor current sense, sampled by internal ADC */
template <>
LogResult LoggerProxy::log<CurrentSenseData>(const CurrentSenseData& t)
{


    tm_repository.adc_tm.timestamp = t.timestamp;
    tm_repository.adc_tm.current_sense_1 = t.current_1_value;
    tm_repository.adc_tm.current_sense_2 = t.current_2_value;

    return logger.log(t);
}

/* ADIS imu */
template <>
LogResult LoggerProxy::log<ADIS16405Data>(const ADIS16405Data& t)
{


    tm_repository.adis_tm.timestamp = miosix::getTick();
    tm_repository.adis_tm.acc_x = t.xaccl_out;
    tm_repository.adis_tm.acc_y = t.yaccl_out;
    tm_repository.adis_tm.acc_z = t.zaccl_out;
    tm_repository.adis_tm.gyro_x = t.xgyro_out;
    tm_repository.adis_tm.gyro_y = t.ygyro_out;
    tm_repository.adis_tm.gyro_z = t.zgyro_out;
    tm_repository.adis_tm.compass_x = t.xmagn_out;
    tm_repository.adis_tm.compass_y = t.ymagn_out;
    tm_repository.adis_tm.compass_z = t.zmagn_out;
    tm_repository.adis_tm.temp = t.temp_out;
    tm_repository.adis_tm.supply_out = t.supply_out;
    tm_repository.adis_tm.aux_adc = t.aux_adc;

    return logger.log(t);
}

/* MPU imu */
template <>
LogResult LoggerProxy::log<MPU9250Data>(const MPU9250Data& t)
{


    tm_repository.mpu_tm.timestamp = miosix::getTick();
    tm_repository.mpu_tm.acc_x = t.accel.getX();
    tm_repository.mpu_tm.acc_y = t.accel.getY();
    tm_repository.mpu_tm.acc_z = t.accel.getZ();
    tm_repository.mpu_tm.gyro_x = t.gyro.getX();
    tm_repository.mpu_tm.gyro_y = t.gyro.getY();
    tm_repository.mpu_tm.gyro_z = t.gyro.getZ();
    tm_repository.mpu_tm.compass_x = t.compass.getX();
    tm_repository.mpu_tm.compass_y = t.compass.getY();
    tm_repository.mpu_tm.compass_z = t.compass.getZ();
    tm_repository.mpu_tm.temp = t.temp;

    return logger.log(t);
}

/* GPS */
template <>
LogResult LoggerProxy::log<GPSData>(const GPSData& t)
{


    // GPS_TM
    tm_repository.gps_tm.timestamp = t.timestamp;
    tm_repository.gps_tm.lat = t.latitude;
    tm_repository.gps_tm.lon = t.longitude;
    tm_repository.gps_tm.altitude = t.height;
    tm_repository.gps_tm.vel_north = t.velocityNorth;
    tm_repository.gps_tm.vel_east = t.velocityEast;
    tm_repository.gps_tm.vel_down = t.velocityDown;
    tm_repository.gps_tm.vel_mag = t.speed;
    tm_repository.gps_tm.fix = (uint8_t) t.fix;
    tm_repository.gps_tm.n_satellites = t.numSatellites;

    // LR_TM
    tm_repository.lr_tm.gps_alt = t.height;
    tm_repository.lr_tm.gps_vel_mag = t.speed;

    // POS_TM
    tm_repository.pos_tm.lat = t.latitude;
    tm_repository.pos_tm.lon = t.longitude;

    return logger.log(t);
}

/* Sensor Manager Tasks */
// Sorry for function-like macro, too lazy to copy-paste
#define UPDATE_TASK(n) \
            tm_repository.sm_task##n##_tm.task_##n##_id = t.id; \
            tm_repository.sm_task##n##_tm.task_##n##_min_value = t.activationStats.minValue; \
            tm_repository.sm_task##n##_tm.task_##n##_max_value = t.activationStats.maxValue; \
            tm_repository.sm_task##n##_tm.task_##n##_mean_value = t.activationStats.mean; \
            tm_repository.sm_task##n##_tm.task_##n##_stddev = t.activationStats.stdev;
template <>
LogResult LoggerProxy::log<TaskStatResult>(const TaskStatResult& t)
{


    switch (t.id)
    {
        case 1:
            UPDATE_TASK(1);
            break;
        case 2:
            UPDATE_TASK(2);
            break;
        case 3:
            UPDATE_TASK(3);
            break;
        case 4:
            UPDATE_TASK(4);
            break;
        case 5:
            UPDATE_TASK(5);
            break;
        default:
            break;
    }

    return logger.log(t);
}

}
