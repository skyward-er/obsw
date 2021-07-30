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
#include <configs/TMTCConfig.h>

#include "DeathStack.h"
#include "FlightStatsRecorder/FlightStatsRecorder.h"
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
//     //     LowRateTMPacker packer(tm_repository.tm_repository.lr_tm.payload);

//     //     packer.packLiftoffTs(tm_repository.lr_tm.liftoff_ts, 0);
//     //     packer.packLiftoffMaxAccTs(tm_repository.lr_tm.liftoff_max_acc_ts,
//     0);
//     //     packer.packLiftoffMaxAcc(tm_repository.lr_tm.liftoff_max_acc_ts,
//     0);
//     //     packer.packMaxZspeedTs(tm_repository.lr_tm.max_zspeed_ts, 0);
//     //     packer.packMaxZspeed(tm_repository.lr_tm.max_zspeed, 0);
//     // packer.packMaxSpeedAltitude(tm_repository.lr_tm.max_speed_altitude,
//     0);
//     //     packer.packApogeeTs(tm_repository.lr_tm.apogee_ts, 0);
//     //     packer.packNxpMinPressure(tm_repository.lr_tm.nxp_min_pressure,
//     0);
//     //     packer.packHwMinPressure(tm_repository.lr_tm.hw_min_pressure, 0);
//     // packer.packKalmanMinPressure(tm_repository.lr_tm.kalman_min_pressure,
//     0);
//     //
//     packer.packDigitalMinPressure(tm_repository.lr_tm.digital_min_pressure,
//     0);
//     // packer.packBaroMaxAltitutde(tm_repository.lr_tm.baro_max_altitutde,
//     0);
//     //     packer.packGpsMaxAltitude(tm_repository.lr_tm.gps_max_altitude,
//     0);
//     //     packer.packApogeeLat(tm_repository.lr_tm.apogee_lat, 0);
//     //     packer.packApogeeLon(tm_repository.lr_tm.apogee_lon, 0);
//     //     packer.packDrogueDplTs(tm_repository.lr_tm.drogue_dpl_ts, 0);
//     //     packer.packDrogueDplMaxAcc(tm_repository.lr_tm.drogue_dpl_max_acc,
//     0);
//     //     packer.packMainDplTs(tm_repository.lr_tm.main_dpl_ts, 0);
//     //     packer.packMainDplAltitude(tm_repository.lr_tm.main_dpl_altitude,
//     0);
//     //     packer.packMainDplZspeed(tm_repository.lr_tm.main_dpl_zspeed, 0);
//     //     packer.packMainDplAcc(tm_repository.lr_tm.main_dpl_acc, 0);
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
        case MavTMList::MAV_PIN_OBS_TM_ID:
            tm_repository.pin_obs_tm.timestamp = miosix::getTick();
            mavlink_msg_pin_obs_tm_encode(sys_id, comp_id, &m,
                                          &(tm_repository.pin_obs_tm));
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
        case MavTMList::MAV_TASK_STATS_TM_ID:
            tm_repository.task_stats_tm.timestamp = miosix::getTick();
            mavlink_msg_task_stats_tm_encode(sys_id, comp_id, &m,
                                             &(tm_repository.task_stats_tm));
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
        case MavTMList::MAV_ABK_TM_ID:
            tm_repository.abk_tm.timestamp = miosix::getTick();
            mavlink_msg_abk_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.abk_tm));
            break;
        case MavTMList::MAV_NAS_TM_ID:
            tm_repository.nas_tm.timestamp = miosix::getTick();
            mavlink_msg_nas_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.nas_tm));
            break;
        case MavTMList::MAV_CAN_TM_ID:
            // tm_repository.can_tm.timestamp = miosix::getTick();
            mavlink_msg_can_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.can_tm));
            break;
        case MavTMList::MAV_ADC_TM_ID:
            tm_repository.adc_tm.timestamp = miosix::getTick();
            mavlink_msg_adc_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.adc_tm));
            break;
        case MavTMList::MAV_MS5803_TM_ID:
            tm_repository.digital_baro_tm.timestamp = miosix::getTick();
            mavlink_msg_ms5803_tm_encode(sys_id, comp_id, &m,
                                         &(tm_repository.digital_baro_tm));
            break;
        case MavTMList::MAV_BMX160_TM_ID:
            tm_repository.bmx_tm.timestamp = miosix::getTick();
            mavlink_msg_bmx160_tm_encode(sys_id, comp_id, &m,
                                         &(tm_repository.bmx_tm));
            break;
        case MavTMList::MAV_LIS3MDL_TM_ID:
            tm_repository.lis3mdl_tm.timestamp = miosix::getTick();
            mavlink_msg_lis3mdl_tm_encode(sys_id, comp_id, &m,
                                          &(tm_repository.lis3mdl_tm));
            break;
        case MavTMList::MAV_GPS_TM_ID:
            tm_repository.gps_tm.timestamp = miosix::getTick();
            mavlink_msg_gps_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.gps_tm));
            break;
        case MavTMList::MAV_STRAIN_BOARD_TM_ID:
            tm_repository.strain_board_tm.timestamp = miosix::getTick();
            mavlink_msg_strain_board_tm_encode(
                sys_id, comp_id, &m, &(tm_repository.strain_board_tm));
            break;
        case MavTMList::MAV_HR_TM_ID:
            tm_repository.hr_tm.timestamp = miosix::getTick();
            mavlink_msg_hr_tm_encode(sys_id, comp_id, &m,
                                     &(tm_repository.hr_tm));
            break;
        case MavTMList::MAV_LR_TM_ID:
            // tm_repository.tm_repository.lr_tm.timestamp = miosix::getTick();
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
            LOG_INFO(log, "Unknown telemetry id: %d\n", req_tm);
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
void TmRepository::update<AeroBrakesControllerStatus>(
    const AeroBrakesControllerStatus& t)
{
    tm_repository.abk_tm.state = (uint8_t)t.state;

    tm_repository.hr_tm.ab_state = (uint8_t)t.state;
}

template <>
void TmRepository::update<AeroBrakesData>(const AeroBrakesData& t)
{
    tm_repository.wind_tm.ab_angle = t.servo_position;

    tm_repository.abk_tm.servo_position = t.servo_position;
    tm_repository.abk_tm.estimated_cd   = t.estimated_cd;
    tm_repository.abk_tm.pid_error      = t.pid_error;

    tm_repository.hr_tm.ab_angle        = t.servo_position;
    tm_repository.hr_tm.ab_estimated_cd = t.estimated_cd;
}

template <>
void TmRepository::update<AeroBrakesAlgorithmData>(
    const AeroBrakesAlgorithmData& t)
{
    tm_repository.abk_tm.z     = t.z;
    tm_repository.abk_tm.vz    = t.vz;
    tm_repository.abk_tm.v_mod = t.vMod;
}

template <>
void TmRepository::update<WindData>(const WindData& t)
{
    tm_repository.wind_tm.wind_speed = t.wind;
}

template <>
void TmRepository::update<CurrentSensorData>(const CurrentSensorData& t)
{
    if (t.channel_id == DeathStackBoard::SensorConfigs::ADC_CS_CUTTER_PRIMARY)
    {
        tm_repository.sensors_tm.c_sense_1 = t.current;
        tm_repository.hr_tm.csense1        = t.current;
        tm_repository.adc_tm.csense1       = t.current;
    }
    else if (t.channel_id ==
             DeathStackBoard::SensorConfigs::ADC_CS_CUTTER_BACKUP)
    {
        tm_repository.sensors_tm.c_sense_2 = t.current;
        tm_repository.hr_tm.csense2        = t.current;
        tm_repository.adc_tm.csense2       = t.current;
    }

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<BatteryVoltageSensorData>(
    const BatteryVoltageSensorData& t)
{
    if (t.channel_id == DeathStackBoard::SensorConfigs::ADC_BATTERY_VOLTAGE)
    {
        tm_repository.hr_tm.vbat         = t.bat_voltage;
        tm_repository.sensors_tm.vbat    = t.bat_voltage;
        tm_repository.adc_tm.bat_voltage = t.bat_voltage;
    }
}

template <>
void TmRepository::update<ADS1118Data>(const ADS1118Data& t)
{
    if (t.channel_id == SensorConfigs::ADC_CH_VREF)
    {
        // tm_repository.wind_tm.pressure_dpl = t.voltage;
        tm_repository.sensors_tm.vbat_5v    = t.voltage;
        tm_repository.hr_tm.vbat_5v         = t.voltage;
        tm_repository.adc_tm.bat_voltage_5v = t.voltage;
    }
}

template <>
void TmRepository::update<MS5803Data>(const MS5803Data& t)
{
    tm_repository.wind_tm.pressure_digital = t.press;
    tm_repository.sensors_tm.ms5803_press  = t.press;
    tm_repository.sensors_tm.ms5803_temp   = t.temp;

    tm_repository.digital_baro_tm.pressure    = t.press;
    tm_repository.digital_baro_tm.temperature = t.temp;

    tm_repository.hr_tm.pressure_digi = t.press;
    tm_repository.hr_tm.temperature   = t.temp;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<MPXHZ6130AData>(const MPXHZ6130AData& t)
{
    tm_repository.wind_tm.pressure_static = t.press;
    tm_repository.sensors_tm.static_press = t.press;
    tm_repository.adc_tm.static_pressure  = t.press;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<SSCDRRN015PDAData>(const SSCDRRN015PDAData& t)
{
    tm_repository.wind_tm.pressure_differential = t.press;
    tm_repository.sensors_tm.pitot_press        = t.press;
    tm_repository.adc_tm.pitot_pressure         = t.press;

    // tm_repository.hr_tm.airspeed_pitot = ?;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<SSCDANN030PAAData>(const SSCDANN030PAAData& t)
{
    tm_repository.wind_tm.pressure_dpl = t.press;
    tm_repository.sensors_tm.dpl_press = t.press;
    tm_repository.hr_tm.pressure_dpl   = t.press;
    tm_repository.adc_tm.dpl_pressure  = t.press;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<BMX160WithCorrectionData>(
    const BMX160WithCorrectionData& t)
{
    tm_repository.sensors_tm.bmx160_acc_x  = t.accel_x;
    tm_repository.sensors_tm.bmx160_acc_y  = t.accel_y;
    tm_repository.sensors_tm.bmx160_acc_z  = t.accel_z;
    tm_repository.sensors_tm.bmx160_gyro_x = t.gyro_x;
    tm_repository.sensors_tm.bmx160_gyro_y = t.gyro_y;
    tm_repository.sensors_tm.bmx160_gyro_z = t.gyro_z;
    tm_repository.sensors_tm.bmx160_mag_x  = t.mag_x;
    tm_repository.sensors_tm.bmx160_mag_y  = t.mag_y;
    tm_repository.sensors_tm.bmx160_mag_z  = t.mag_z;

    tm_repository.bmx_tm.acc_x  = t.accel_x;
    tm_repository.bmx_tm.acc_y  = t.accel_y;
    tm_repository.bmx_tm.acc_z  = t.accel_z;
    tm_repository.bmx_tm.gyro_x = t.gyro_x;
    tm_repository.bmx_tm.gyro_y = t.gyro_y;
    tm_repository.bmx_tm.gyro_z = t.gyro_z;
    tm_repository.bmx_tm.mag_x  = t.mag_x;
    tm_repository.bmx_tm.mag_y  = t.mag_y;
    tm_repository.bmx_tm.mag_z  = t.mag_z;

    tm_repository.hr_tm.acc_x  = t.accel_x;
    tm_repository.hr_tm.acc_y  = t.accel_y;
    tm_repository.hr_tm.acc_z  = t.accel_z;
    tm_repository.hr_tm.gyro_x = t.gyro_x;
    tm_repository.hr_tm.gyro_y = t.gyro_y;
    tm_repository.hr_tm.gyro_z = t.gyro_z;
    tm_repository.hr_tm.mag_x  = t.mag_x;
    tm_repository.hr_tm.mag_y  = t.mag_y;
    tm_repository.hr_tm.mag_z  = t.mag_z;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<BMX160Temperature>(const BMX160Temperature& t)
{
    tm_repository.sensors_tm.bmx160_temp = t.temp;
    tm_repository.bmx_tm.temp            = t.temp;
}

template <>
void TmRepository::update<LIS3MDLData>(const LIS3MDLData& t)
{
    tm_repository.sensors_tm.lis3mdl_mag_x = t.mag_x;
    tm_repository.sensors_tm.lis3mdl_mag_y = t.mag_y;
    tm_repository.sensors_tm.lis3mdl_mag_z = t.mag_z;
    tm_repository.sensors_tm.lis3mdl_temp  = t.temp;

    tm_repository.lis3mdl_tm.mag_x = t.mag_x;
    tm_repository.lis3mdl_tm.mag_y = t.mag_y;
    tm_repository.lis3mdl_tm.mag_z = t.mag_z;
    tm_repository.lis3mdl_tm.temp  = t.temp;
}

/* GPS */
template <>
void TmRepository::update<UbloxGPSData>(const UbloxGPSData& t)
{
    // GPS_TM
    tm_repository.gps_tm.latitude     = t.latitude;
    tm_repository.gps_tm.longitude    = t.longitude;
    tm_repository.gps_tm.height       = t.height;
    tm_repository.gps_tm.vel_north    = t.velocity_north;
    tm_repository.gps_tm.vel_east     = t.velocity_east;
    tm_repository.gps_tm.vel_down     = t.velocity_down;
    tm_repository.gps_tm.speed        = t.speed;
    tm_repository.gps_tm.fix          = (uint8_t)t.fix;
    tm_repository.gps_tm.track        = t.track;
    tm_repository.gps_tm.n_satellites = t.num_satellites;

    // HR TM
    tm_repository.hr_tm.gps_lat = t.latitude;
    tm_repository.hr_tm.gps_lon = t.longitude;
    tm_repository.hr_tm.gps_alt = t.height;
    tm_repository.hr_tm.gps_fix = (uint8_t)t.fix;

    // TEST TM
    tm_repository.test_tm.gps_nsats = t.num_satellites;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
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

template <>
void TmRepository::update<DeathStackStatus>(const DeathStackStatus& t)
{
    tm_repository.sys_tm.death_stack    = t.death_stack;
    tm_repository.sys_tm.logger         = t.logger;
    tm_repository.sys_tm.ev_broker      = t.ev_broker;
    tm_repository.sys_tm.pin_obs        = t.pin_obs;
    tm_repository.sys_tm.sensors        = t.sensors;
    tm_repository.sys_tm.radio          = t.radio;
    tm_repository.sys_tm.state_machines = t.state_machines;
}

/* Flight Mode Manager */
template <>
void TmRepository::update<FMMStatus>(const FMMStatus& t)
{
    tm_repository.fmm_tm.state    = static_cast<uint8_t>(t.state);
    tm_repository.hr_tm.fmm_state = static_cast<uint8_t>(t.state);
}

template <>
void TmRepository::update<NASStatus>(const NASStatus& t)
{
    tm_repository.nas_tm.state    = static_cast<uint8_t>(t.state);
    tm_repository.hr_tm.nas_state = static_cast<uint8_t>(t.state);
}

template <>
void TmRepository::update<NASKalmanState>(const NASKalmanState& t)
{
    Vector3f orientation = t.toEul();

    tm_repository.nas_tm.x0    = t.x0;
    tm_repository.nas_tm.x1    = t.x1;
    tm_repository.nas_tm.x2    = t.x2;
    tm_repository.nas_tm.x3    = t.x3;
    tm_repository.nas_tm.x4    = t.x4;
    tm_repository.nas_tm.x5    = t.x5;
    tm_repository.nas_tm.x6    = t.x6;
    tm_repository.nas_tm.x7    = t.x7;
    tm_repository.nas_tm.x8    = t.x8;
    tm_repository.nas_tm.x9    = t.x9;
    tm_repository.nas_tm.x10   = t.x10;
    tm_repository.nas_tm.x11   = t.x11;
    tm_repository.nas_tm.x12   = t.x12;
    tm_repository.nas_tm.roll  = orientation(0);
    tm_repository.nas_tm.pitch = orientation(1);
    tm_repository.nas_tm.yaw   = orientation(2);
    // tm_repository.nas_tm.triad_x  = ...
    // tm_repository.nas_tm.triad_y = ...
    // tm_repository.nas_tm.triad_z   = ...

    tm_repository.hr_tm.nas_x = t.x0;
    tm_repository.hr_tm.nas_y = t.x1;
    tm_repository.hr_tm.nas_z =
        -t.x2;  // Negative sign because we're working in the NED
                // frame but we want a positive altitude as output
    tm_repository.hr_tm.nas_vx    = t.x3;
    tm_repository.hr_tm.nas_vy    = t.x4;
    tm_repository.hr_tm.nas_vz    = t.x5;
    tm_repository.hr_tm.nas_roll  = orientation(0);
    tm_repository.hr_tm.nas_pitch = orientation(1);
    tm_repository.hr_tm.nas_yaw   = orientation(2);
    tm_repository.hr_tm.nas_bias0 = t.x10;
    tm_repository.hr_tm.nas_bias1 = t.x11;
    tm_repository.hr_tm.nas_bias2 = t.x12;
}

template <>
void TmRepository::update<NASReferenceValues>(const NASReferenceValues& t)
{
    tm_repository.nas_tm.ref_accel_x = t.ref_accel_x;
    tm_repository.nas_tm.ref_accel_y = t.ref_accel_y;
    tm_repository.nas_tm.ref_accel_z = t.ref_accel_z;
    tm_repository.nas_tm.ref_mag_x   = t.ref_mag_x;
    tm_repository.nas_tm.ref_mag_y   = t.ref_mag_y;
    tm_repository.nas_tm.ref_mag_z   = t.ref_mag_z;

    tm_repository.nas_tm.ref_latitude  = t.ref_latitude;
    tm_repository.nas_tm.ref_longitude = t.ref_longitude;

    tm_repository.nas_tm.ref_pressure = t.ref_pressure;
}

/* Launch and Nosecone detachment pins */
template <>
void TmRepository::update<PinStatus>(const PinStatus& t)
{
    switch (t.pin)
    {

        case ObservedPin::LAUNCH:
        {
            tm_repository.pin_obs_tm.pin_launch_last_change =
                t.last_state_change / 1000;
            tm_repository.pin_obs_tm.pin_launch_num_changes =
                t.num_state_changes;
            tm_repository.pin_obs_tm.pin_launch_state = t.state;
            // HR TM
            tm_repository.hr_tm.pin_launch = t.state;
            break;
        }
        case ObservedPin::NOSECONE:
        {
            tm_repository.pin_obs_tm.pin_nosecone_last_change =
                t.last_state_change / 1000;
            tm_repository.pin_obs_tm.pin_nosecone_num_changes =
                t.num_state_changes;
            tm_repository.pin_obs_tm.pin_nosecone_state = t.state;

            // HR TM
            tm_repository.hr_tm.pin_nosecone = t.state;
            break;
        }
        case ObservedPin::DPL_SERVO:
        {
            tm_repository.pin_obs_tm.pin_dpl_servo_last_change =
                t.last_state_change / 1000;
            tm_repository.pin_obs_tm.pin_dpl_servo_num_changes =
                t.num_state_changes;
            tm_repository.pin_obs_tm.pin_dpl_servo_state = t.state;

            // HR TM
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

/* TMTCManager (Mavlink) */
template <>
void TmRepository::update<MavlinkStatus>(const MavlinkStatus& t)
{
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

// /* Sensor Manager */
// template <>
// void TmRepository::update<SensorManagerStatus>(const SensorManagerStatus& t)
// {
//     tm_repository.sm_tm.sensor_state = t.sensor_status;
//     tm_repository.sm_tm.state        = (uint8_t)t.state;
// }

// /* Deployment Controller */
template <>
void TmRepository::update<DeploymentStatus>(const DeploymentStatus& t)
{
    tm_repository.dpl_tm.fsm_state = (uint8_t)t.state;
    tm_repository.dpl_tm.primary_cutter_state =
        (uint8_t)t.primary_cutter_state.state;
    tm_repository.dpl_tm.backup_cutter_state =
        (uint8_t)t.backup_cutter_state.state;
    tm_repository.dpl_tm.servo_position = t.servo_position;

    // HR TM
    tm_repository.hr_tm.dpl_state = (int)t.state;
}

/* ADA state machine */
template <>
void TmRepository::update<ADAControllerStatus>(const ADAControllerStatus& t)
{
    tm_repository.ada_tm.state                = (uint8_t)t.state;
    tm_repository.ada_tm.apogee_reached       = t.apogee_reached;
    tm_repository.ada_tm.aerobrakes_disabled  = t.disable_aerobrakes;
    tm_repository.ada_tm.dpl_altitude_reached = t.dpl_altitude_reached;

    tm_repository.hr_tm.ada_state = (uint8_t)t.state;
}

/* ADA target dpl pressure */
template <>
void TmRepository::update<TargetDeploymentAltitude>(
    const TargetDeploymentAltitude& t)
{
    tm_repository.ada_tm.target_dpl_altitude = t.deployment_altitude;
}

/* ADA kalman filter values */
template <>
void TmRepository::update<ADAKalmanState>(const ADAKalmanState& t)
{
    tm_repository.ada_tm.kalman_x0 = t.x0;
    tm_repository.ada_tm.kalman_x1 = t.x1;
    tm_repository.ada_tm.kalman_x2 = t.x2;

    // tm_repository.ada_tm.kalman_acc_x0 = t.x0_acc;
    // tm_repository.ada_tm.kalman_acc_x1 = t.x1_acc;
    // tm_repository.ada_tm.kalman_acc_x2 = t.x2_acc;

    // HR_TM
    tm_repository.hr_tm.pressure_ada = t.x0;
    // tm_repository.hr_tm.vert_accel = t.x2;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

// /* ADA kalman altitude values */
template <>
void TmRepository::update<ADAData>(const ADAData& t)
{
    tm_repository.ada_tm.msl_altitude = t.msl_altitude;
    tm_repository.ada_tm.vert_speed   = t.vert_speed;

    tm_repository.hr_tm.msl_altitude = t.msl_altitude;
    tm_repository.hr_tm.vert_speed   = t.vert_speed;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<ADAReferenceValues>(const ADAReferenceValues& t)
{
    tm_repository.ada_tm.msl_pressure    = t.msl_pressure;
    tm_repository.ada_tm.msl_temperature = t.msl_temperature;

    tm_repository.ada_tm.ref_altitude    = t.ref_altitude;
    tm_repository.ada_tm.ref_pressure    = t.ref_pressure;
    tm_repository.ada_tm.ref_temperature = t.ref_temperature;
}

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

template <>
void TmRepository::update<SystemData>(const SystemData& t)
{
    tm_repository.lr_tm.cpu_load  = t.cpu_usage;
    tm_repository.lr_tm.free_heap = t.free_heap;
}

template <>
void TmRepository::update<LiftOffStats>(const LiftOffStats& t)
{
    tm_repository.lr_tm.liftoff_ts = t.T_liftoff;

    tm_repository.lr_tm.liftoff_max_acc_ts = t.T_max_acc;
    tm_repository.lr_tm.liftoff_max_acc    = t.acc_max;

    tm_repository.lr_tm.max_z_speed_ts     = t.T_max_speed;
    tm_repository.lr_tm.max_z_speed        = t.vert_speed_max;
    tm_repository.lr_tm.max_airspeed_pitot = t.airspeed_pitot_max;
    tm_repository.lr_tm.max_speed_altitude = t.altitude_max_speed;
}

template <>
void TmRepository::update<ApogeeStats>(const ApogeeStats& t)
{
    tm_repository.lr_tm.apogee_ts = t.T_apogee;

    tm_repository.lr_tm.static_min_pressure  = t.static_min_pressure;
    tm_repository.lr_tm.digital_min_pressure = t.digital_min_pressure;
    tm_repository.lr_tm.ada_min_pressure     = t.ada_min_pressure;
    tm_repository.lr_tm.digital_min_pressure = t.digital_min_pressure;

    tm_repository.lr_tm.baro_max_altitutde = t.baro_max_altitude;
    tm_repository.lr_tm.gps_max_altitude   = t.gps_max_altitude;

    tm_repository.lr_tm.apogee_lat = t.lat_apogee;
    tm_repository.lr_tm.apogee_lon = t.lon_apogee;
}

template <>
void TmRepository::update<DrogueDPLStats>(const DrogueDPLStats& t)
{
    tm_repository.lr_tm.drogue_dpl_ts         = t.T_dpl;
    tm_repository.lr_tm.drogue_dpl_max_acc    = t.max_dpl_acc;
    tm_repository.lr_tm.dpl_vane_max_pressure = t.max_dpl_vane_pressure;
}

template <>
void TmRepository::update<MainDPLStats>(const MainDPLStats& t)
{
    tm_repository.lr_tm.main_dpl_altitude_ts = t.T_dpl;
    tm_repository.lr_tm.main_dpl_altitude    = t.altitude_dpl;
    tm_repository.lr_tm.main_dpl_acc         = t.max_dpl_acc;
    tm_repository.lr_tm.main_dpl_zspeed      = t.vert_speed_dpl;
}

template <>
void TmRepository::update<CutterTestStats>(const CutterTestStats& t)
{
    tm_repository.dpl_tm.primary_cutter_test_current = t.cutter_1_avg;
    tm_repository.dpl_tm.backup_cutter_test_current  = t.cutter_2_avg;
}

#ifdef HARDWARE_IN_THE_LOOP
template <>
void TmRepository::update<HILImuData>(const HILImuData& t)
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

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<HILBaroData>(const HILBaroData& t)
{
    tm_repository.wind_tm.pressure_digital = t.press;
    tm_repository.sensors_tm.ms5803_press  = t.press;

    tm_repository.hr_tm.pressure_digi = t.press;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}

template <>
void TmRepository::update<HILGpsData>(const HILGpsData& t)
{
    // GPS_TM
    tm_repository.gps_tm.latitude     = t.latitude;
    tm_repository.gps_tm.longitude    = t.longitude;
    tm_repository.gps_tm.height       = t.height;
    tm_repository.gps_tm.vel_north    = t.velocity_north;
    tm_repository.gps_tm.vel_east     = t.velocity_east;
    tm_repository.gps_tm.vel_down     = t.velocity_down;
    tm_repository.gps_tm.speed        = t.speed;
    tm_repository.gps_tm.fix          = (uint8_t)t.fix;
    tm_repository.gps_tm.track        = t.track;
    tm_repository.gps_tm.n_satellites = t.num_satellites;

    // HR TM
    tm_repository.hr_tm.gps_lat = t.latitude;
    tm_repository.hr_tm.gps_lon = t.longitude;
    tm_repository.hr_tm.gps_alt = t.height;
    tm_repository.hr_tm.gps_fix = (uint8_t)t.fix;

    // TEST TM
    tm_repository.test_tm.gps_nsats = t.num_satellites;

    DeathStack::getInstance()->state_machines->flight_stats->update(t);
}
#endif

}  // namespace DeathStackBoard