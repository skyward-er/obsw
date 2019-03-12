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
#include <DeathStack/configs/TMTCConfig.h>
#include <DeathStack/Status.h>

namespace DeathStackBoard
{
namespace TMBuilder
{

using Status::tm_repository;
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
         case MavTMList::MAV_HM1_TM_ID:
            mavlink_msg_hm1_tm_encode(sys_id, comp_id, &m, &(tm_repository.hm1_tm));
            break;
         case MavTMList::MAV_IGN_TM_ID:
            mavlink_msg_ign_tm_encode(sys_id, comp_id, &m, &(tm_repository.ign_tm));
            break;
         case MavTMList::MAV_HR_TM_ID:
            mavlink_msg_hr_tm_encode(sys_id, comp_id, &m, &(tm_repository.hr_tm));
            break;
         case MavTMList::MAV_LR_TM_ID:
            mavlink_msg_lr_tm_encode(sys_id, comp_id, &m, &(tm_repository.lr_tm));
            break;
         case MavTMList::MAV_LOGGER_TM_ID:
            mavlink_msg_logger_tm_encode(sys_id, comp_id, &m, &(tm_repository.logger_tm));
            break;
         case MavTMList::MAV_TMTC_TM_ID:
            mavlink_msg_tmtc_tm_encode(sys_id, comp_id, &m, &(tm_repository.tmtc_tm));
            break;
         case MavTMList::MAV_SM_TM_ID:
            mavlink_msg_sm_tm_encode(sys_id, comp_id, &m, &(tm_repository.sm_tm));
            break;
         case MavTMList::MAV_IGN_CTRL_TM_ID:
            mavlink_msg_ign_ctrl_tm_encode(sys_id, comp_id, &m, &(tm_repository.ign_ctrl_tm));
            break;
         case MavTMList::MAV_DPL_CTRL_TM_ID:
            mavlink_msg_dpl_ctrl_tm_encode(sys_id, comp_id, &m, &(tm_repository.dpl_ctrl_tm));
            break;
         case MavTMList::MAV_ADA_TM_ID:
            mavlink_msg_ada_tm_encode(sys_id, comp_id, &m, &(tm_repository.ada_tm));
            break;
         case MavTMList::MAV_CAN_TM_ID:
            mavlink_msg_can_tm_encode(sys_id, comp_id, &m, &(tm_repository.can_tm));
            break;
         case MavTMList::MAV_AD7994_TM_ID:
            mavlink_msg_ad7994_tm_encode(sys_id, comp_id, &m, &(tm_repository.ad7994_tm));
            break;
         case MavTMList::MAV_ADC_TM_ID:
            mavlink_msg_adc_tm_encode(sys_id, comp_id, &m, &(tm_repository.adc_tm));
            break;
         case MavTMList::MAV_ADIS_TM_ID:
            mavlink_msg_adis_tm_encode(sys_id, comp_id, &m, &(tm_repository.adis_tm));
            break;
         case MavTMList::MAV_MPU_TM_ID:
            mavlink_msg_mpu_tm_encode(sys_id, comp_id, &m, &(tm_repository.mpu_tm));
            break;
         case MavTMList::MAV_GPS_TM_ID:
            mavlink_msg_gps_tm_encode(sys_id, comp_id, &m, &(tm_repository.gps_tm));
            break;
         case MavTMList::MAV_POS_TM_ID:
            mavlink_msg_pos_tm_encode(sys_id, comp_id, &m, &(tm_repository.pos_tm));
            break;
         case MavTMList::MAV_SM_TASK1_TM_ID:
            mavlink_msg_sm_task1_tm_encode(sys_id, comp_id, &m, &(tm_repository.sm_task1_tm));
            break;
         case MavTMList::MAV_SM_TASK2_TM_ID:
            mavlink_msg_sm_task2_tm_encode(sys_id, comp_id, &m, &(tm_repository.sm_task2_tm));
            break;
         case MavTMList::MAV_SM_TASK3_TM_ID:
            mavlink_msg_sm_task3_tm_encode(sys_id, comp_id, &m, &(tm_repository.sm_task3_tm));
            break;
         case MavTMList::MAV_SM_TASK4_TM_ID:
            mavlink_msg_sm_task4_tm_encode(sys_id, comp_id, &m, &(tm_repository.sm_task4_tm));
            break;
         case MavTMList::MAV_SM_TASK5_TM_ID:
            mavlink_msg_sm_task5_tm_encode(sys_id, comp_id, &m, &(tm_repository.sm_task5_tm));
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
static mavlink_message_t buildTelemetry(uint8_t req_tm) 
{
    miosix::PauseKernelLock kLock;
    return getTM(req_tm, TMTC_MAV_SYSID, TMTC_MAV_SYSID);
}

} /* namespace TMBuilder */
} /* namespace DeathStackBoard */
