/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#include "TmRepository.h"
// #include <bitpacking/hermes/HermesPackets.h>
#include <Debug.h>
// #include <diagnostic/PrintLogger.h>
#include <configs/TMTCConfig.h>

#include "LoggerService/LoggerService.h"
#include "configs/SensorManagerConfig.h"
namespace DeathStackBoard
{
/* Periodic TM updaters */

// bool TmRepository::updateHR()
// {
//     //     HighRateTMPacker packer(tm_repository.hr_tm.payload);

//     //     packer.packTimestamp(miosix::getTick(), curHrIndex);

//     //     packer.packPressureAda(hr_pkt.pressure_ada, curHrIndex);
//     //     packer.packPressureDigi(hr_pkt.pressure_digi, curHrIndex);
//     //     packer.packMslAltitude(hr_pkt.msl_altitude, curHrIndex);
//     //     packer.packAglAltitude(hr_pkt.agl_altitude, curHrIndex);
//     //     packer.packVertSpeed(hr_pkt.vert_speed, curHrIndex);
//     //     packer.packVertSpeed2(hr_pkt.vert_speed_2, curHrIndex);
//     //     packer.packAccX(hr_pkt.acc_x, curHrIndex);
//     //     packer.packAccY(hr_pkt.acc_y, curHrIndex);
//     //     packer.packAccZ(hr_pkt.acc_z, curHrIndex);
//     //     packer.packGyroX(hr_pkt.gyro_x, curHrIndex);
//     //     packer.packGyroY(hr_pkt.gyro_y, curHrIndex);
//     //     packer.packGyroZ(hr_pkt.gyro_z, curHrIndex);
//     //     packer.packGpsLat(hr_pkt.gps_lat, curHrIndex);
//     //     packer.packGpsLon(hr_pkt.gps_lon, curHrIndex);
//     //     packer.packGpsAlt(hr_pkt.gps_alt, curHrIndex);
//     //     packer.packGpsFix(hr_pkt.gps_fix, curHrIndex);
//     //     packer.packTemperature(hr_pkt.temperature, curHrIndex);
//     //     packer.packFmmState(hr_pkt.fmm_state, curHrIndex);
//     //     packer.packDplState(hr_pkt.dpl_state, curHrIndex);
//     //     packer.packPinLaunch(hr_pkt.pin_launch, curHrIndex);
//     //     packer.packPinNosecone(hr_pkt.pin_nosecone, curHrIndex);

//     //     curHrIndex = (curHrIndex + 1) % N_PKT_HR;

//     //     if (curHrIndex == 0)
//     //         return true;
//     //     else
//     return false;
// }

// void TmRepository::updateLR()
// {
//     //     LowRateTMPacker packer(tm_repository.lr_tm.payload);

//     //     packer.packLiftoffTs(lr_pkt.liftoff_ts, 0);
//     //     packer.packLiftoffMaxAccTs(lr_pkt.liftoff_max_acc_ts, 0);
//     //     packer.packLiftoffMaxAcc(lr_pkt.liftoff_max_acc_ts, 0);
//     //     packer.packMaxZspeedTs(lr_pkt.max_zspeed_ts, 0);
//     //     packer.packMaxZspeed(lr_pkt.max_zspeed, 0);
//     //     packer.packMaxSpeedAltitude(lr_pkt.max_speed_altitude, 0);
//     //     packer.packApogeeTs(lr_pkt.apogee_ts, 0);
//     //     packer.packNxpMinPressure(lr_pkt.nxp_min_pressure, 0);
//     //     packer.packHwMinPressure(lr_pkt.hw_min_pressure, 0);
//     //     packer.packKalmanMinPressure(lr_pkt.kalman_min_pressure, 0);
//     //     packer.packDigitalMinPressure(lr_pkt.digital_min_pressure, 0);
//     //     packer.packBaroMaxAltitutde(lr_pkt.baro_max_altitutde, 0);
//     //     packer.packGpsMaxAltitude(lr_pkt.gps_max_altitude, 0);
//     //     packer.packApogeeLat(lr_pkt.apogee_lat, 0);
//     //     packer.packApogeeLon(lr_pkt.apogee_lon, 0);
//     //     packer.packDrogueDplTs(lr_pkt.drogue_dpl_ts, 0);
//     //     packer.packDrogueDplMaxAcc(lr_pkt.drogue_dpl_max_acc, 0);
//     //     packer.packMainDplTs(lr_pkt.main_dpl_ts, 0);
//     //     packer.packMainDplAltitude(lr_pkt.main_dpl_altitude, 0);
//     //     packer.packMainDplZspeed(lr_pkt.main_dpl_zspeed, 0);
//     //     packer.packMainDplAcc(lr_pkt.main_dpl_acc, 0);
// }

/* TM getter */

mavlink_message_t TmRepository::packTM(uint8_t req_tm, uint8_t sys_id,
                                       uint8_t comp_id)
{
    mavlink_message_t m;
    mavlink_nack_tm_t nack_tm;

    miosix::PauseKernelLock kLock;

    switch (req_tm)
    {
        case MavTMList::MAV_SYS_TM_ID:
            tm_repository.sys_tm.timestamp = miosix::getTick();
            mavlink_msg_sys_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.sys_tm));
            break;
        case MavTMList::MAV_FMM_TM_ID:
            tm_repository.fmm_tm.timestamp = miosix::getTick();
            mavlink_msg_fmm_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.fmm_tm));
            break;
        case MavTMList::MAV_LOGGER_TM_ID:
            tm_repository.logger_tm.timestamp = miosix::getTick();
            mavlink_msg_logger_tm_encode(sys_id, comp_id, &m,
                                         &(tm_repository.logger_tm));
            break;
        case MavTMList::MAV_TMTC_TM_ID:
            tm_repository.tmtc_tm.timestamp = miosix::getTick();
            mavlink_msg_tmtc_tm_encode(sys_id, comp_id, &m,
                                       &(tm_repository.tmtc_tm));
            break;
        case MavTMList::MAV_SM_TM_ID:
            tm_repository.sm_tm.timestamp = miosix::getTick();
            mavlink_msg_sm_tm_encode(sys_id, comp_id, &m,
                                     &(tm_repository.sm_tm));
            break;
        case MavTMList::MAV_DPL_TM_ID:
            tm_repository.dpl_tm.timestamp = miosix::getTick();
            mavlink_msg_dpl_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.dpl_tm));
            break;
        case MavTMList::MAV_ADA_TM_ID:
            tm_repository.ada_tm.timestamp = miosix::getTick();
            mavlink_msg_ada_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.ada_tm));
            break;
        case MavTMList::MAV_ADC_TM_ID:
            tm_repository.adc_tm.timestamp = miosix::getTick();
            mavlink_msg_adc_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.adc_tm));
            break;
        case MavTMList::MAV_GPS_TM_ID:
            tm_repository.gps_tm.timestamp = miosix::getTick();
            mavlink_msg_gps_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.gps_tm));
            break;
        case MavTMList::MAV_HR_TM_ID:
            mavlink_msg_hr_tm_encode(sys_id, comp_id, &m,
                                     &(tm_repository.hr_tm));
            break;
        case MavTMList::MAV_LR_TM_ID:
            mavlink_msg_lr_tm_encode(sys_id, comp_id, &m,
                                     &(tm_repository.lr_tm));
            break;
        case MavTMList::MAV_TEST_TM_ID:
            tm_repository.test_tm.timestamp = miosix::getTick();
            mavlink_msg_test_tm_encode(sys_id, comp_id, &m,
                                       &(tm_repository.test_tm));
            break;
        case MavTMList::MAV_WINDTUNNEL_TM_ID:
            tm_repository.wind_tm.timestamp = miosix::getTick();
            mavlink_msg_windtunnel_tm_encode(sys_id, comp_id, &m,
                                             &(tm_repository.wind_tm));
            break;
        case MavTMList::MAV_SENSORS_TM_ID:
            tm_repository.sensors_tm.timestamp = miosix::getTick();
            mavlink_msg_sensors_tm_encode(sys_id, comp_id, &m,
                                             &(tm_repository.sensors_tm));
            break;
        default:
        {
            // PrintLogger log = Logging::getLogger("deathstack.tmrepo");
            TRACE("[MAV] Unknown telemetry id: %d\n", req_tm);
            nack_tm.recv_msgid = 0;
            nack_tm.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(sys_id, comp_id, &m, &nack_tm);
            break;
        }
    }

    return m;
}

/* Components TM upaters */

template <>
void TmRepository::update<AeroBrakesData>(const AeroBrakesData& t)
{
    tm_repository.wind_tm.ab_angle = t.servo_position;
    tm_repository.hr_tm.ab_angle = t.servo_position;
}

template <>
void TmRepository::update<WindData>(const WindData& t)
{
    tm_repository.wind_tm.wind_speed = t.wind;
}

template <>
void TmRepository::update<ADS1118Data>(const ADS1118Data& t)
{
    if (t.channel_id == DeathStackBoard::SensorConfigs::ADC_CH_VREF){
        tm_repository.wind_tm.pressure_dpl = t.voltage;
        tm_repository.sensors_tm.v_bat = t.voltage;
    }
}

template <>
void TmRepository::update<MS5803Data>(const MS5803Data& t)
{
    tm_repository.wind_tm.pressure_digital = t.press;
    tm_repository.sensors_tm.ms5803_press  = t.press;
    tm_repository.sensors_tm.ms5803_temp   = t.temp;

    tm_repository.hr_tm.pressure_digi = t.press;
}

template <>
void TmRepository::update<MPXHZ6130AData>(const MPXHZ6130AData& t)
{
    tm_repository.wind_tm.pressure_static = t.press;
    tm_repository.sensors_tm.static_press = t.press;
}

template <>
void TmRepository::update<SSCDRRN015PDAData>(const SSCDRRN015PDAData& t)
{
    tm_repository.wind_tm.pressure_differential = t.press;
    tm_repository.sensors_tm.pitot_press        = t.press;
}

template <>
void TmRepository::update<SSCDANN030PAAData>(const SSCDANN030PAAData& t)
{
    tm_repository.wind_tm.pressure_dpl = t.press;
    tm_repository.sensors_tm.dpl_press = t.press;
    tm_repository.hr_tm.pressure_dpl = t.press;
}

template <>
void TmRepository::update<BMX160Data>(const BMX160Data& t)
{
    tm_repository.sensors_tm.bmx160_acc_x = t.accel_x;
    tm_repository.sensors_tm.bmx160_acc_y = t.accel_y;
    tm_repository.sensors_tm.bmx160_acc_z = t.accel_z;

    tm_repository.sensors_tm.bmx160_gyro_x = t.gyro_x;
    tm_repository.sensors_tm.bmx160_gyro_y = t.gyro_y;
    tm_repository.sensors_tm.bmx160_gyro_z = t.gyro_z;

    tm_repository.sensors_tm.bmx160_mag_x = t.mag_x;
    tm_repository.sensors_tm.bmx160_mag_y = t.mag_y;
    tm_repository.sensors_tm.bmx160_mag_z = t.mag_z;

    tm_repository.hr_tm.acc_x = t.accel_x;
    tm_repository.hr_tm.acc_y = t.accel_y;
    tm_repository.hr_tm.acc_z = t.accel_z;

    tm_repository.hr_tm.gyro_x = t.gyro_x;
    tm_repository.hr_tm.gyro_y = t.gyro_y;
    tm_repository.hr_tm.gyro_z = t.gyro_z;

    tm_repository.hr_tm.mag_x = t.mag_x;
    tm_repository.hr_tm.mag_y = t.mag_y;
    tm_repository.hr_tm.mag_z = t.mag_z;
}

template <>
void TmRepository::update<BMX160Temerature>(const BMX160Temerature& t)
{
    tm_repository.sensors_tm.bmx160_temp = t.temp;
    tm_repository.hr_tm.temperature = t.temp;
}

template <>
void TmRepository::update<LIS3MDLData>(const LIS3MDLData& t)
{
    tm_repository.sensors_tm.lis3mdl_mag_x = t.mag_x;
    tm_repository.sensors_tm.lis3mdl_mag_y = t.mag_y;
    tm_repository.sensors_tm.lis3mdl_mag_z = t.mag_z;

    tm_repository.sensors_tm.lis3mdl_temp = t.temp;
}

template <>
void TmRepository::update<UbloxGPSData>(const UbloxGPSData& t)
{
    tm_repository.hr_tm.gps_lat = t.latitude;
    tm_repository.hr_tm.gps_lon = t.longitude;
    tm_repository.hr_tm.gps_alt = t.height;
    tm_repository.hr_tm.gps_fix = (uint8_t)t.fix;
}

template <>
void TmRepository::update<Xbee::ATCommandResponseFrameLog>(
    const Xbee::ATCommandResponseFrameLog& t)
{
    if (strncmp(t.at_command, "DB", 2) == 0 && t.command_data_length == 1)
    {
        tm_repository.wind_tm.last_RSSI = -t.command_data[0];
    }
}

// template <>
// void TmRepository::update<BoardStatus>(const BoardStatus& t)
// {
//     // tm_repository.sys_tm.board          = t.death_stack;
//     tm_repository.sys_tm.logger         = t.logger;
//     tm_repository.sys_tm.ev_broker      = t.ev_broker;
//     tm_repository.sys_tm.pin_obs        = t.pin_obs;
//     tm_repository.sys_tm.fmm            = t.pin_obs;
//     tm_repository.sys_tm.sensor_manager = t.sensor_manager;
//     tm_repository.sys_tm.ada            = t.ada;
//     tm_repository.sys_tm.tmtc           = t.tmtc;
//     tm_repository.sys_tm.ign            = t.ign;
//     tm_repository.sys_tm.dpl            = t.dpl;
// }

// /* Flight Mode Manager */
// template <>
// void TmRepository::update<FMMStatus>(const FMMStatus& t)
// {
//     tm_repository.fmm_tm.state = static_cast<uint8_t>(t.state);
//     hr_pkt.fmm_state           = static_cast<uint8_t>(t.state);
// }

/* Launch and Nosecone detachment pins */
template <>
void TmRepository::update<PinStatus>(const PinStatus& t)
{
    switch (t.pin)
    {

        case ObservedPin::LAUNCH:
        {
            // tm_repository.fmm_tm.pin_launch_last_change =
            //     t.last_state_change / 1000;
            // tm_repository.fmm_tm.pin_launch_num_changes =
            // t.num_state_changes; tm_repository.fmm_tm.pin_launch_state =
            // t.state;
            // HR TM
            tm_repository.hr_tm.pin_launch = t.state;
            break;
        }
        case ObservedPin::NOSECONE:
        {
            // tm_repository.fmm_tm.pin_nosecone_last_change =
            //     t.last_state_change / 1000;
            // tm_repository.fmm_tm.pin_nosecone_num_changes =
            // t.num_state_changes; tm_repository.fmm_tm.pin_nosecone_state =
            // t.state;

            // HR TM
            tm_repository.hr_tm.pin_nosecone = t.state;
            break;
        }
        case ObservedPin::DPL_SERVO:
        {
            // No time to change telemetries, since we have no GPS use gps
            // telemetry to send motor pin state.
            // tm_repository.gps_tm.n_satellites = t.num_state_changes;
            // tm_repository.gps_tm.lat          = t.last_state_change / 1000;
            // hr_pkt.gps_fix                    = t.state;

            tm_repository.hr_tm.servo_sensor = t.state;
            break;
        }
        default:
            break;
    }
}

/* Logger */
template <>
void TmRepository::update<LogStats>(const LogStats& t)
{
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

    tm_repository.wind_tm.log_num    = t.logNumber;
    tm_repository.wind_tm.log_status = t.opened ? t.statWriteError : -1000;

    tm_repository.hr_tm.logger_error = t.opened ? t.statWriteError : 255;
}

// /* TMTCManager (Mavlink) */
// template <>
// void TmRepository::update<MavlinkStatus>(const MavlinkStatus& t)
// {
//     // mavchannel stats
//     tm_repository.tmtc_tm.n_send_queue   = t.n_send_queue;
//     tm_repository.tmtc_tm.max_send_queue = t.max_send_queue;
//     tm_repository.tmtc_tm.n_send_errors  = t.n_send_errors;
//     tm_repository.tmtc_tm.msg_received   = t.mav_stats.msg_received;
//     // mav stats
//     tm_repository.tmtc_tm.buffer_overrun = t.mav_stats.buffer_overrun;
//     tm_repository.tmtc_tm.parse_error    = t.mav_stats.parse_error;
//     tm_repository.tmtc_tm.parse_state    = t.mav_stats.parse_state;
//     tm_repository.tmtc_tm.packet_idx     = t.mav_stats.packet_idx;
//     tm_repository.tmtc_tm.current_rx_seq = t.mav_stats.current_rx_seq;
//     tm_repository.tmtc_tm.current_tx_seq = t.mav_stats.current_tx_seq;
//     tm_repository.tmtc_tm.packet_rx_success_count =
//         t.mav_stats.packet_rx_success_count;
//     tm_repository.tmtc_tm.packet_rx_drop_count =
//         t.mav_stats.packet_rx_drop_count;
// }

// /* Sensor Manager */
// template <>
// void TmRepository::update<SensorManagerStatus>(const SensorManagerStatus& t)
// {
//     tm_repository.sm_tm.sensor_state = t.sensor_status;
//     tm_repository.sm_tm.state        = (uint8_t)t.state;
// }

// /* Deployment Controller */
// template <>
// void TmRepository::update<DeploymentStatus>(const DeploymentStatus& t)
// {
//     tm_repository.dpl_tm.fsm_state    = (uint8_t)t.state;
//     tm_repository.dpl_tm.cutter_state = (uint8_t)t.cutter_status.state;

//     // HR TM
//     hr_pkt.dpl_state = t.state;
// }

// /* ADA state machine */
// template <>
// void TmRepository::update<ADAControllerStatus>(const ADAControllerStatus& t)
// {
//     tm_repository.ada_tm.state = (uint8_t)t.state;
// }

// /* ADA target dpl pressure */
// template <>
// void TmRepository::update<TargetDeploymentAltitude>(
//     const TargetDeploymentAltitude& t)
// {
//     tm_repository.ada_tm.target_dpl_altitude = t.deployment_altitude;
// }

// /* ADA kalman filter values */
// template <>
// void TmRepository::update<KalmanState>(const KalmanState& t)
// {
//     tm_repository.ada_tm.kalman_x0 = t.x0;
//     tm_repository.ada_tm.kalman_x1 = t.x1;
//     tm_repository.ada_tm.kalman_x2 = t.x2;

//     tm_repository.ada_tm.kalman_acc_x0 = t.x0_acc;
//     tm_repository.ada_tm.kalman_acc_x1 = t.x1_acc;
//     tm_repository.ada_tm.kalman_acc_x2 = t.x2_acc;
// }

// /* ADA kalman altitude values */
template <>
void TmRepository::update<ADAData>(const ADAData& t)
{
    tm_repository.hr_tm.msl_altitude = t.msl_altitude;
    tm_repository.hr_tm.vert_speed = t.vert_speed;
}

// template <>
// void TmRepository::update<ReferenceValues>(const ReferenceValues& t)
// {
//     tm_repository.ada_tm.msl_pressure    = t.msl_pressure;
//     tm_repository.ada_tm.msl_temperature = t.msl_temperature;

//     tm_repository.ada_tm.ref_altitude    = t.ref_altitude;
//     tm_repository.ada_tm.ref_pressure    = t.ref_pressure;
//     tm_repository.ada_tm.ref_temperature = t.ref_temperature;
// }

// /* Battery status, sampled by internal ADC */
// template <>
// void TmRepository::update<BatteryVoltageData>(const BatteryVoltageData& t)
// {
//     tm_repository.adc_tm.battery_voltage = t.volt;
//     tm_repository.test_tm.battery_volt   = t.volt;
// }

// /* Current sense, sampled by internal ADC */
// template <>
// void TmRepository::update<CurrentSenseData>(const CurrentSenseData& t)
// {
//     tm_repository.adc_tm.current_sense_1 = t.current_1;
//     tm_repository.adc_tm.current_sense_2 = t.current_2;

//     tm_repository.test_tm.th_cut_1 = t.current_1;
//     tm_repository.test_tm.th_cut_2 = t.current_2;
// }

// template <>
// void TmRepository::update<MS5803Data>(const MS5803Data& t)
// {
//     hr_pkt.pressure_digi = t.pressure;
// }

// /* GPS */
// template <>
// void TmRepository::update<PiksiData>(const PiksiData& t)
// {
//     // GPS_TM
//     // tm_repository.gps_tm.lat          = t.gps_data.latitude;
//     // tm_repository.gps_tm.lon          = t.gps_data.longitude;
//     // tm_repository.gps_tm.altitude     = t.gps_data.height;
//     // tm_repository.gps_tm.vel_north    = t.gps_data.velocityNorth;
//     // tm_repository.gps_tm.vel_east     = t.gps_data.velocityEast;
//     // tm_repository.gps_tm.vel_down     = t.gps_data.velocityDown;
//     // tm_repository.gps_tm.vel_mag      = t.gps_data.speed;
//     // tm_repository.gps_tm.fix          = (uint8_t)t.fix;
//     // tm_repository.gps_tm.n_satellites = t.gps_data.numSatellites;

//     // HR TM
//     hr_pkt.gps_lat = t.gps_data.latitude;
//     hr_pkt.gps_lon = t.gps_data.longitude;
//     hr_pkt.gps_alt = t.gps_data.height;
//     // hr_pkt.gps_fix = t.fix;

//     // TEST TM
//     tm_repository.test_tm.gps_nsats = t.gps_data.numSatellites;
// }

// template <>
// void TmRepository::update<TaskStatResult>(const TaskStatResult& t)
// {
//     switch (t.id)
//     {
//         case static_cast<uint8_t>(SensorSamplerId::SIMPLE_20HZ):
//             tm_repository.sm_tm.task_20hz_max    = t.periodStats.maxValue;
//             tm_repository.sm_tm.task_20hz_min    = t.periodStats.minValue;
//             tm_repository.sm_tm.task_20hz_mean   = t.periodStats.mean;
//             tm_repository.sm_tm.task_20hz_stddev = t.periodStats.stdev;
//             break;
//         case static_cast<uint8_t>(SensorSamplerId::GPS):
//             tm_repository.sm_tm.task_10hz_max    = t.periodStats.maxValue;
//             tm_repository.sm_tm.task_10hz_min    = t.periodStats.minValue;
//             tm_repository.sm_tm.task_10hz_mean   = t.periodStats.mean;
//             tm_repository.sm_tm.task_10hz_stddev = t.periodStats.stdev;
//             break;
//         case static_cast<uint8_t>(SensorSamplerId::SIMPLE_250HZ):
//             tm_repository.sm_tm.task_250hz_max    = t.periodStats.maxValue;
//             tm_repository.sm_tm.task_250hz_min    = t.periodStats.minValue;
//             tm_repository.sm_tm.task_250hz_mean   = t.periodStats.mean;
//             tm_repository.sm_tm.task_250hz_stddev = t.periodStats.stdev;
//             break;

//         default:
//             break;
//     }
// }

// template <>
// void TmRepository::update<LiftOffStats>(const LiftOffStats& t)
// {
//     lr_pkt.liftoff_ts = t.T_liftoff;

//     lr_pkt.liftoff_max_acc_ts = t.T_max_acc;
//     lr_pkt.liftoff_max_acc    = t.acc_max;

//     lr_pkt.max_zspeed_ts      = t.T_max_speed;
//     lr_pkt.max_zspeed         = t.vert_speed_max;
//     lr_pkt.max_speed_altitude = t.altitude_max_speed;
// }

// template <>
// void TmRepository::update<ApogeeStats>(const ApogeeStats& t)
// {
//     lr_pkt.apogee_ts = t.T_apogee;

//     lr_pkt.nxp_min_pressure     = t.nxp_min_pressure;
//     lr_pkt.hw_min_pressure      = t.hw_min_pressure;
//     lr_pkt.kalman_min_pressure  = t.kalman_min_pressure;
//     lr_pkt.digital_min_pressure = t.digital_min_pressure;

//     lr_pkt.baro_max_altitutde = t.baro_max_altitude;
//     lr_pkt.gps_max_altitude   = t.gps_max_altitude;

//     lr_pkt.apogee_lat = t.lat_apogee;
//     lr_pkt.apogee_lon = t.lon_apogee;
// }

// template <>
// void TmRepository::update<DrogueDPLStats>(const DrogueDPLStats& t)
// {
//     lr_pkt.drogue_dpl_ts      = t.T_dpl;
//     lr_pkt.drogue_dpl_max_acc = t.max_dpl_acc;
// }

// template <>
// void TmRepository::update<CutterTestStats>(const CutterTestStats& t)
// {
//     tm_repository.dpl_tm.cutter_1_test_current = t.cutter_1_avg;
//     tm_repository.dpl_tm.cutter_2_test_current = t.cutter_2_avg;
// }

}  // namespace DeathStackBoard