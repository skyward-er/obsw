/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#include <Common.h>
#include <DeathStack/LogProxy/Telemetries.h>
#include <DeathStack/configs/TMTCConfig.h>

namespace DeathStackBoard
{
namespace TMBuilder
{

/**
 * Retrieve one of the telemetry structs
 * @param req_tm    required telemetry
 * @param sys_id    system id to pack it with
 * @param comp_id   component id to pack it with
 * @return          packed mavlink struct of that telemetry
 */
static mavlink_message_t getTM(uint8_t req_tm, uint8_t sys_id, uint8_t comp_id)
{
    mavlink_message_t m;

    switch (req_tm)
    {
        case MavTMList::MAV_SYS_TM_ID:
            mavlink_msg_sys_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.sys_tm));
            break;
        case MavTMList::MAV_FMM_TM_ID:
            mavlink_msg_fmm_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.fmm_tm));
            break;
        case MavTMList::MAV_LOGGER_TM_ID:
            mavlink_msg_logger_tm_encode(sys_id, comp_id, &m,
                                         &(tm_repository.logger_tm));
            break;
        case MavTMList::MAV_TMTC_TM_ID:
            mavlink_msg_tmtc_tm_encode(sys_id, comp_id, &m,
                                       &(tm_repository.tmtc_tm));
            break;
        case MavTMList::MAV_SM_TM_ID:
            mavlink_msg_sm_tm_encode(sys_id, comp_id, &m,
                                     &(tm_repository.sm_tm));
            break;
        case MavTMList::MAV_DPL_TM_ID:
            mavlink_msg_dpl_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.dpl_tm));
            break;
        case MavTMList::MAV_IGN_TM_ID:
            mavlink_msg_ign_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.ign_tm));
            break;
        case MavTMList::MAV_ADA_TM_ID:
            mavlink_msg_ada_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.ada_tm));
            break;
        case MavTMList::MAV_CAN_TM_ID:
            mavlink_msg_can_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.can_tm));
            break;
        case MavTMList::MAV_ADC_TM_ID:
            mavlink_msg_adc_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.adc_tm));
            break;
        case MavTMList::MAV_ADIS_TM_ID:
            mavlink_msg_adis_tm_encode(sys_id, comp_id, &m,
                                       &(tm_repository.adis_tm));
            break;
        case MavTMList::MAV_MPU_TM_ID:
            mavlink_msg_mpu_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.mpu_tm));
            break;
        case MavTMList::MAV_GPS_TM_ID:
            mavlink_msg_gps_tm_encode(sys_id, comp_id, &m,
                                      &(tm_repository.gps_tm));
            break;

        case MavTMList::MAV_HR_TM_ID:
            tm_repository.hr_tm.timestamp = miosix::getTick();
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
        default:
            // TODO: manage error
            break;
    }

    return m;
}

/**
 * Synchronously read the corresponding telemetry from the statusRepo
 * (aka last logged struct).
 * @param req_tm     requested telemetry
 * @return           the telemetry as mavlink_message_t
 */
inline mavlink_message_t buildTelemetry(uint8_t req_tm)
{
    miosix::PauseKernelLock kLock;
    return getTM(req_tm, TMTC_MAV_SYSID, TMTC_MAV_SYSID);
}

} /* namespace TMBuilder */
} /* namespace DeathStackBoard */
