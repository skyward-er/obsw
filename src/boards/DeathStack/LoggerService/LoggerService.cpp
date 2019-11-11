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
#include "LoggerService.h"
#include "FlightStatsRecorder.h"
#include "TmRepository.h"

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack/DeathStackStatus.h"
#include "DeathStack/DeploymentController/DeploymentData.h"
#include "DeathStack/FlightModeManager/FMMStatus.h"
#include "DeathStack/IgnitionController/IgnitionStatus.h"
#include "DeathStack/PinHandler/PinHandlerData.h"
#include "DeathStack/SensorManager/SensorManagerData.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapperData.h"
#include "DeathStack/SensorManager/Sensors/PiksiData.h"

#include "drivers/canbus/CanUtils.h"
#include "drivers/mavlink/MavlinkStatus.h"
#include "scheduler/TaskSchedulerData.h"
#include "sensors/ADIS16405/ADIS16405Data.h"
#include "sensors/MPU9250/MPU9250Data.h"

namespace DeathStackBoard
{
/**
 * @brief Each function here is an implementation of the log() method for a
 * specific status struct or class.
 * The rationale is that, whenever a status struct is logged, the corresponding
 * telemetry struct is updated synchronously in the telemetry repo.
 * Telemetry requests from ground will therefore return the latest logged struct
 * from each component.
 */

template <>
LogResult LoggerService::log<DeathStackStatus>(const DeathStackStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.sys_tm.death_stack    = t.death_stack;
        tm_repository.sys_tm.logger         = t.logger;
        tm_repository.sys_tm.ev_broker      = t.ev_broker;
        tm_repository.sys_tm.pin_obs        = t.pin_obs;
        tm_repository.sys_tm.fmm            = t.pin_obs;
        tm_repository.sys_tm.sensor_manager = t.sensor_manager;
        tm_repository.sys_tm.ada            = t.ada;
        tm_repository.sys_tm.tmtc           = t.tmtc;
        tm_repository.sys_tm.ign            = t.ign;
        tm_repository.sys_tm.dpl            = t.dpl;
    }
    return logger.log(t);
}

/* Flight Mode Manager */
template <>
LogResult LoggerService::log<FMMStatus>(const FMMStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.fmm_tm.state    = static_cast<uint8_t>(t.state);
        tm_repository.hr_tm.fmm_state = static_cast<uint8_t>(t.state);
    }
    return logger.log(t);
}

/* Launch and Nosecone detachment pins */
template <>
LogResult LoggerService::log<PinStatus>(const PinStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        switch (t.pin)
        {

            case ObservedPin::LAUNCH:
            {
                tm_repository.fmm_tm.pin_launch_last_change =
                    t.last_state_change;
                tm_repository.fmm_tm.pin_launch_num_changes =
                    t.num_state_changes;
                tm_repository.fmm_tm.pin_launch_state = t.state;
                // HR TM
                tm_repository.hr_tm.pin_launch = t.state;
                break;
            }
            case ObservedPin::NOSECONE:
            {
                tm_repository.fmm_tm.pin_nosecone_last_change =
                    t.last_state_change;
                tm_repository.fmm_tm.pin_nosecone_num_changes =
                    t.num_state_changes;
                tm_repository.fmm_tm.pin_nosecone_state = t.state;

                // HR TM
                tm_repository.hr_tm.pin_nosecone = t.state;
                break;
            }
            default:
                break;
        }
    }
    return logger.log(t);
}

/* Ignition Board */
template <>
LogResult LoggerService::log<IgnBoardLoggableStatus>(
    const IgnBoardLoggableStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        uint16_t bs;
        memcpy(&bs, &t.board_status, sizeof(bs));

        tm_repository.ign_tm.avr_bitfield = bs & 0xFF;
        tm_repository.ign_tm.avr_bitfield = bs >> 8;
    }

    return logger.log(t);
}

/* Ignition Controller */
template <>
LogResult LoggerService::log<IgnCtrlStatus>(const IgnCtrlStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.ign_tm.fsm_state  = t.fsm_state;
        tm_repository.ign_tm.last_event = t.last_event;

        tm_repository.ign_tm.n_rcv_message   = t.n_rcv_messages;
        tm_repository.ign_tm.n_sent_messages = t.n_sent_messages;

        // Bitfield
        tm_repository.ign_tm.cmd_bitfield = 0;
        tm_repository.ign_tm.cmd_bitfield |= t.launch_sent;
        tm_repository.ign_tm.cmd_bitfield |= (t.abort_sent << 1);
        tm_repository.ign_tm.cmd_bitfield |= (t.abort_rcv << 2);
    }
    return logger.log(t);
}

/* Logger */
template <>
LogResult LoggerService::log<LogStats>(const LogStats& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.logger_tm.statLogNumber       = t.logNumber;
        tm_repository.logger_tm.statTooLargeSamples = t.statTooLargeSamples;
        tm_repository.logger_tm.statDroppedSamples  = t.statDroppedSamples;
        tm_repository.logger_tm.statQueuedSamples   = t.statQueuedSamples;
        tm_repository.logger_tm.statBufferFilled    = t.statBufferFilled;
        tm_repository.logger_tm.statBufferWritten   = t.statBufferWritten;
        tm_repository.logger_tm.statWriteFailed     = t.statWriteFailed;
        tm_repository.logger_tm.statWriteError      = t.statWriteError;
        tm_repository.logger_tm.statWriteTime       = t.statWriteTime;
        tm_repository.logger_tm.statMaxWriteTime    = t.statMaxWriteTime;
    }
    return logger.log(t);
}

/* TMTCManager (Mavlink) */
template <>
LogResult LoggerService::log<MavlinkStatus>(const MavlinkStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        // mavchannel stats
        tm_repository.tmtc_tm.n_send_queue   = t.n_send_queue;
        tm_repository.tmtc_tm.max_send_queue = t.max_send_queue;
        tm_repository.tmtc_tm.n_send_errors  = t.n_send_errors;
        tm_repository.tmtc_tm.msg_received   = t.mav_stats.msg_received;
        // mav stats
        tm_repository.tmtc_tm.buffer_overrun = t.mav_stats.buffer_overrun;
        tm_repository.tmtc_tm.parse_error    = t.mav_stats.parse_error;
        tm_repository.tmtc_tm.parse_state    = t.mav_stats.parse_state;
        tm_repository.tmtc_tm.packet_idx     = t.mav_stats.packet_idx;
        tm_repository.tmtc_tm.current_rx_seq = t.mav_stats.current_rx_seq;
        tm_repository.tmtc_tm.current_tx_seq = t.mav_stats.current_tx_seq;
        tm_repository.tmtc_tm.packet_rx_success_count =
            t.mav_stats.packet_rx_success_count;
        tm_repository.tmtc_tm.packet_rx_drop_count =
            t.mav_stats.packet_rx_drop_count;
    }
    return logger.log(t);
}

/* Sensor Manager */
template <>
LogResult LoggerService::log<SensorManagerStatus>(const SensorManagerStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.sm_tm.sensor_state = t.sensor_status;
        tm_repository.sm_tm.state        = (uint8_t)t.state;
    }
    return logger.log(t);
}

/* Deployment Controller */
template <>
LogResult LoggerService::log<DeploymentStatus>(const DeploymentStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.dpl_tm.fsm_state    = (uint8_t)t.state;
        tm_repository.dpl_tm.cutter_state = (uint8_t)t.cutter_status.state;

        // HR TM
        tm_repository.hr_tm.dpl_state = t.state;
    }
    return logger.log(t);
}

/* ADA state machine */
template <>
LogResult LoggerService::log<ADAControllerStatus>(const ADAControllerStatus& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.ada_tm.state = (uint8_t)t.state;
    }
    return logger.log(t);
}

/* ADA target dpl pressure */
template <>
LogResult LoggerService::log<TargetDeploymentAltitude>(
    const TargetDeploymentAltitude& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.ada_tm.target_dpl_altitude = t.deployment_altitude;
    }
    return logger.log(t);
}

/* ADA kalman filter values */
template <>
LogResult LoggerService::log<KalmanState>(const KalmanState& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.ada_tm.kalman_x0 = t.x0;
        tm_repository.ada_tm.kalman_x1 = t.x1;
        tm_repository.ada_tm.kalman_x2 = t.x2;

        tm_repository.ada_tm.kalman_acc_x0 = t.x0_acc;
        tm_repository.ada_tm.kalman_acc_x1 = t.x1_acc;
        tm_repository.ada_tm.kalman_acc_x2 = t.x2_acc;
    }

    flight_stats.update(t);

    return logger.log(t);
}

/* ADA kalman altitude values */
template <>
LogResult LoggerService::log<ADAData>(const ADAData& t)
{
    {
        miosix::PauseKernelLock kLock;
        tm_repository.hr_tm.msl_altitude = t.msl_altitude;
        tm_repository.hr_tm.agl_altitude = t.dpl_altitude;
        tm_repository.hr_tm.vert_speed   = t.vert_speed;
        tm_repository.hr_tm.vert_speed_2 = t.acc_vert_speed;
    }
    flight_stats.update(t);

    return logger.log(t);
}

template <>
LogResult LoggerService::log<ReferenceValues>(const ReferenceValues& t)
{
    {
        miosix::PauseKernelLock kLock;
        tm_repository.ada_tm.msl_pressure    = t.msl_pressure;
        tm_repository.ada_tm.msl_temperature = t.msl_temperature;

        tm_repository.ada_tm.ref_altitude    = t.ref_altitude;
        tm_repository.ada_tm.ref_pressure    = t.ref_pressure;
        tm_repository.ada_tm.ref_temperature = t.ref_temperature;
    }

    return logger.log(t);
}

// template <>
// LogResult LoggerService::log<ADACalibrationData>(const ADACalibrationData& t)
// {
//     {
//         miosix::PauseKernelLock kLock;

//         tm_repository.ada_tm.ref_pressure_mean     = t.pressure_calib.mean;
//         tm_repository.ada_tm.ref_pressure_stddev   = t.pressure_calib.stdev;
//         tm_repository.ada_tm.ref_pressure_nsamples =
//         t.pressure_calib.nSamples;
//     }
//     return logger.log(t);
// }

/* Canbus stats */
template <>
LogResult LoggerService::log<CanStatus>(const CanStatus& t)
{
    {
        miosix::PauseKernelLock kLock;
        tm_repository.can_tm.n_sent       = t.n_sent;
        tm_repository.can_tm.n_rcv        = t.n_rcv;
        tm_repository.can_tm.last_sent    = t.last_sent;
        tm_repository.can_tm.last_rcv     = t.last_rcv;
        tm_repository.can_tm.last_sent_ts = t.last_sent_ts;
        tm_repository.can_tm.last_rcv_ts  = t.last_rcv_ts;
    }

    return logger.log(t);
}

/* Main Barometer */
template <>
LogResult LoggerService::log<AD7994WrapperData>(const AD7994WrapperData& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.adc_tm.nxp_baro_volt     = t.nxp_baro_volt;
        tm_repository.adc_tm.nxp_baro_flag     = t.nxp_baro_flag;
        tm_repository.adc_tm.nxp_baro_pressure = t.nxp_baro_pressure;

        tm_repository.adc_tm.hw_baro_volt     = t.honeywell_baro_volt;
        tm_repository.adc_tm.hw_baro_flag     = t.honeywell_baro_flag;
        tm_repository.adc_tm.hw_baro_pressure = t.honeywell_baro_pressure;

        // HR TM
        tm_repository.hr_tm.pressure_ada = t.nxp_baro_pressure;

        // Test tm
        tm_repository.test_tm.pressure_hw = t.honeywell_baro_pressure;
    }

    flight_stats.update(t);
    return logger.log(t);
}

/* Battery status, sampled by internal ADC */
template <>
LogResult LoggerService::log<BatteryVoltageData>(const BatteryVoltageData& t)
{
    tm_repository.adc_tm.battery_voltage = t.volt;
    tm_repository.test_tm.battery_volt   = t.volt;

    return logger.log(t);
}

/* Current sense, sampled by internal ADC */
template <>
LogResult LoggerService::log<CurrentSenseData>(const CurrentSenseData& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.adc_tm.current_sense_1 = t.current_1;
        tm_repository.adc_tm.current_sense_2 = t.current_2;

        tm_repository.test_tm.th_cut_1 = t.current_1;
        tm_repository.test_tm.th_cut_2 = t.current_2;
    }
    return logger.log(t);
}

template <>
LogResult LoggerService::log<MS5803Data>(const MS5803Data& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.hr_tm.pressure_digi = t.pressure;
    }
    return logger.log(t);
}

/* ADIS imu */
template <>
LogResult LoggerService::log<ADIS16405Data>(const ADIS16405Data& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.adis_tm.acc_x      = t.xaccl_out;
        tm_repository.adis_tm.acc_y      = t.yaccl_out;
        tm_repository.adis_tm.acc_z      = t.zaccl_out;
        tm_repository.adis_tm.gyro_x     = t.xgyro_out;
        tm_repository.adis_tm.gyro_y     = t.ygyro_out;
        tm_repository.adis_tm.gyro_z     = t.zgyro_out;
        tm_repository.adis_tm.compass_x  = t.xmagn_out;
        tm_repository.adis_tm.compass_y  = t.ymagn_out;
        tm_repository.adis_tm.compass_z  = t.zmagn_out;
        tm_repository.adis_tm.temp       = t.temp_out;
        tm_repository.adis_tm.supply_out = t.supply_out;
        tm_repository.adis_tm.aux_adc    = t.aux_adc;

        // HR TM
        // tm_repository.hr_tm.z_acc = t.zaccl_out;
    }

    return logger.log(t);
}

/* MPU imu */
template <>
LogResult LoggerService::log<MPU9250Data>(const MPU9250Data& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.mpu_tm.acc_x     = t.accel.getX();
        tm_repository.mpu_tm.acc_y     = t.accel.getY();
        tm_repository.mpu_tm.acc_z     = t.accel.getZ();
        tm_repository.mpu_tm.gyro_x    = t.gyro.getX();
        tm_repository.mpu_tm.gyro_y    = t.gyro.getY();
        tm_repository.mpu_tm.gyro_z    = t.gyro.getZ();
        tm_repository.mpu_tm.compass_x = t.compass.getX();
        tm_repository.mpu_tm.compass_y = t.compass.getY();
        tm_repository.mpu_tm.compass_z = t.compass.getZ();
        tm_repository.mpu_tm.temp      = t.temp;

        // HR TM
        tm_repository.hr_tm.acc_x = t.accel.getX();
        tm_repository.hr_tm.acc_y = t.accel.getY();
        tm_repository.hr_tm.acc_z = t.accel.getZ();

        tm_repository.hr_tm.gyro_x = t.gyro.getX();
        tm_repository.hr_tm.gyro_y = t.gyro.getY();
        tm_repository.hr_tm.gyro_z = t.gyro.getZ();
    }

    flight_stats.update(t);
    return logger.log(t);
}

/* LM75b temperature */
template <>
LogResult LoggerService::log<LM75BData>(const LM75BData& t)
{
    {
        miosix::PauseKernelLock kLock;
        tm_repository.test_tm.temp_analog = t.temp_analog;
        tm_repository.hr_tm.temperature   = t.temp_analog;
        tm_repository.test_tm.temp_imu    = t.temp_imu;
    }

    return logger.log(t);
}

/* GPS */
template <>
LogResult LoggerService::log<PiksiData>(const PiksiData& t)
{
    {
        miosix::PauseKernelLock kLock;

        // GPS_TM
        tm_repository.gps_tm.lat          = t.gps_data.latitude;
        tm_repository.gps_tm.lon          = t.gps_data.longitude;
        tm_repository.gps_tm.altitude     = t.gps_data.height;
        tm_repository.gps_tm.vel_north    = t.gps_data.velocityNorth;
        tm_repository.gps_tm.vel_east     = t.gps_data.velocityEast;
        tm_repository.gps_tm.vel_down     = t.gps_data.velocityDown;
        tm_repository.gps_tm.vel_mag      = t.gps_data.speed;
        tm_repository.gps_tm.fix          = (uint8_t)t.fix;
        tm_repository.gps_tm.n_satellites = t.gps_data.numSatellites;

        // HR TM
        tm_repository.hr_tm.gps_lat = t.gps_data.latitude;
        tm_repository.hr_tm.gps_lon = t.gps_data.longitude;
        tm_repository.hr_tm.gps_alt = t.gps_data.height;
        tm_repository.hr_tm.gps_fix = t.fix;

        //TEST TM
        tm_repository.test_tm.gps_nsats = t.gps_data.numSatellites;
    }

    flight_stats.update(t);

    return logger.log(t);
}

template <>
LogResult LoggerService::log<TaskStatResult>(const TaskStatResult& t)
{
    {
        miosix::PauseKernelLock kLock;
        switch (t.id)
        {
            case static_cast<uint8_t>(SensorSamplerId::SIMPLE_20HZ):
                tm_repository.sm_tm.task_20hz_max    = t.periodStats.maxValue;
                tm_repository.sm_tm.task_20hz_min    = t.periodStats.minValue;
                tm_repository.sm_tm.task_20hz_mean   = t.periodStats.mean;
                tm_repository.sm_tm.task_20hz_stddev = t.periodStats.stdev;
                break;
            case static_cast<uint8_t>(SensorSamplerId::GPS):
                tm_repository.sm_tm.task_10hz_max    = t.periodStats.maxValue;
                tm_repository.sm_tm.task_10hz_min    = t.periodStats.minValue;
                tm_repository.sm_tm.task_10hz_mean   = t.periodStats.mean;
                tm_repository.sm_tm.task_10hz_stddev = t.periodStats.stdev;
                break;
            case static_cast<uint8_t>(SensorSamplerId::SIMPLE_250HZ):
                tm_repository.sm_tm.task_250hz_max    = t.periodStats.maxValue;
                tm_repository.sm_tm.task_250hz_min    = t.periodStats.minValue;
                tm_repository.sm_tm.task_250hz_mean   = t.periodStats.mean;
                tm_repository.sm_tm.task_250hz_stddev = t.periodStats.stdev;
                break;

            default:
                break;
        }
    }
    return logger.log(t);
}

template <>
LogResult LoggerService::log<LiftOffStats>(const LiftOffStats& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.lr_tm.liftoff_ts = t.T_liftoff;

        tm_repository.lr_tm.liftoff_max_acc_ts = t.T_max_acc;
        tm_repository.lr_tm.liftoff_max_acc    = t.acc_max;

        tm_repository.lr_tm.max_zspeed_ts      = t.T_max_speed;
        tm_repository.lr_tm.max_zspeed         = t.vert_speed_max;
        tm_repository.lr_tm.max_speed_altitude = t.altitude_max_speed;
    }

    return logger.log(t);
}

template <>
LogResult LoggerService::log<ApogeeStats>(const ApogeeStats& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.lr_tm.apogee_ts = t.T_apogee;

        tm_repository.lr_tm.nxp_min_pressure     = t.nxp_min_pressure;
        tm_repository.lr_tm.hw_min_pressure      = t.hw_min_pressure;
        tm_repository.lr_tm.kalman_min_pressure  = t.kalman_min_pressure;
        tm_repository.lr_tm.digital_min_pressure = t.digital_min_pressure;

        tm_repository.lr_tm.baro_max_altitutde = t.baro_max_altitude;
        tm_repository.lr_tm.gps_max_altitude   = t.gps_max_altitude;

        tm_repository.lr_tm.apogee_lat = t.lat_apogee;
        tm_repository.lr_tm.apogee_lon = t.lon_apogee;
    }

    return logger.log(t);
}

template <>
LogResult LoggerService::log<DrogueDPLStats>(const DrogueDPLStats& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.lr_tm.drogue_dpl_ts      = t.T_dpl;
        tm_repository.lr_tm.drogue_dpl_max_acc = t.max_dpl_acc;
    }

    return logger.log(t);
}

template <>
LogResult LoggerService::log<MainDPLStats>(const MainDPLStats& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.lr_tm.main_dpl_ts       = t.T_dpl;
        tm_repository.lr_tm.main_dpl_acc      = t.max_dpl_acc;
        tm_repository.lr_tm.main_dpl_altitude = t.altitude_dpl;
        tm_repository.lr_tm.main_dpl_zspeed   = t.vert_speed_dpl;
    }
    return logger.log(t);
}

template <>
LogResult LoggerService::log<CutterTestStats>(const CutterTestStats& t)
{
    {
        miosix::PauseKernelLock kLock;

        tm_repository.dpl_tm.cutter_1_test_current = t.cutter_1_avg;
        tm_repository.dpl_tm.cutter_2_test_current = t.cutter_2_avg;
    }

    return logger.log(t);
}

}  // namespace DeathStackBoard
