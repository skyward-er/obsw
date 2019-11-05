/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#pragma once

#include <mavlink_skyward_lib/mavlink_lib/hermes/mavlink.h>

/**
 * @brief This file contains all telemetry packets in the form of mavlink
 * structs. These packets are updated by the LoggerService and read by the
 * TMTCManager.
 *
 * Notice that, if the LoggerService is not active, the value inside tm packets
 * WILL NOT BE UPDATED.
 */
namespace DeathStackBoard
{

struct HighRateTM_t
{
    long long timestamp;
    float pressure_ada;
    float pressure_digi;
    float msl_altitude;
    float agl_altitude;
    float vert_speed;
    float vert_speed_2;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gps_lat;
    float gps_lon;
    float gps_alt;
    float temperature;
    uint8_t fmm_state;
    uint8_t dpl_state;
    uint8_t pin_launch;
    uint8_t pin_nosecone;
    uint8_t gps_fix;
};

struct LowRateTM_t
{
    long long liftoff_ts;
    long long liftoff_max_acc_ts;
    float liftoff_max_acc;
    long long max_zspeed_ts;
    float max_zspeed;
    float max_speed_altitude;
    long long apogee_ts;
    float nxp_min_pressure;
    float hw_min_pressure;
    float kalman_min_pressure;
    float digital_min_pressure;
    float baro_max_altitutde;
    float gps_max_altitude;
    float apogee_lat;
    float apogee_lon;
    long long drogue_dpl_ts;
    float drogue_dpl_max_acc;
    long long main_dpl_ts;
    float main_dpl_altitude;
    float main_dpl_zspeed;
    float main_dpl_acc;
};

/* Struct containing all tms in the form of mavlink structs */
struct TmRepository_t
{
    mavlink_sys_tm_t sys_tm;
    mavlink_fmm_tm_t fmm_tm;
    mavlink_logger_tm_t logger_tm;
    mavlink_tmtc_tm_t tmtc_tm;

    mavlink_sm_tm_t sm_tm;
    mavlink_ign_tm_t ign_tm;
    mavlink_dpl_tm_t dpl_tm;
    mavlink_ada_tm_t ada_tm;

    mavlink_can_tm_t can_tm;

    mavlink_adc_tm_t adc_tm;
    mavlink_adis_tm_t adis_tm;
    mavlink_mpu_tm_t mpu_tm;
    mavlink_gps_tm_t gps_tm;

    HighRateTM_t hr_tm;
    LowRateTM_t lr_tm;

    mavlink_test_tm_t test_tm;
};

/* Forward declaration of the global struct. */
extern TmRepository_t tm_repository;

/**
 * @brief This function initializes all the packets with 0s.
 */
void initTelemetries();

}  // namespace DeathStackBoard