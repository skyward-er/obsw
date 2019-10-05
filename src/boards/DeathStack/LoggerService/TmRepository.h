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
 * @brief This file contains all telemetry packets in the form of mavlink structs.
 * These packets are updated by the LoggerService and read by the TMTCManager.
 * 
 * Notice that, if the LoggerService is not active, the value inside tm packets WILL
 * NOT BE UPDATED.
 */
namespace DeathStackBoard
{

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

    mavlink_hr_tm_t hr_tm;
    mavlink_lr_tm_t lr_tm;

    mavlink_test_tm_t test_tm;
};

/* Forward declaration of the global struct. */
extern TmRepository_t tm_repository;

/**
 * @brief This function initializes all the packets with 0s.
 */
void initTelemetries();

}  // namespace DeathStackBoard