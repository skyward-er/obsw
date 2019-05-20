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

namespace DeathStackBoard
{
/* Enumeration of all possible TMs
enum MavTMList: uint8_t
{
    MAV_HM1_TM_ID,
    MAV_IGN_TM_ID,
    MAV_HR_TM_ID,
    MAV_LR_TM_ID,
    MAV_LOGGER_TM_ID,
    MAV_TMTC_TM_ID,
    MAV_SM_TM_ID,
    MAV_IGN_CTRL_TM_ID,
    MAV_DPL_CTRL_TM_ID,
    MAV_ADA_TM_ID,
    MAV_CAN_TM_ID,
    MAV_AD7994_TM_ID,
    MAV_ADC_TM_ID,
    MAV_ADIS_TM_ID,
    MAV_MPU_TM_ID,
    MAV_GPS_TM_ID,
    MAV_POS_TM_ID,
    MAV_SM_TASK1_TM_ID,
    MAV_SM_TASK2_TM_ID,
    MAV_SM_TASK3_TM_ID,
    MAV_SM_TASK4_TM_ID,
    MAV_SM_TASK5_TM_ID,
}; */

/* Struct containing all tms in the form of mavlink structs */
struct Telemetries
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

    mavlink_hr_tm_t hr_tm;
    mavlink_lr_tm_t lr_tm;

    mavlink_test_tm_t test_tm;
};

extern Telemetries tm_repository;

void initTelemetries();

}  // namespace Status