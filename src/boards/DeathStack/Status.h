/** @file
 *  @brief Skyward Status Repository
 */
#pragma once

#include <skyward-boardcore/libs/mavlink_skyward_lib/mavlink_lib/hermes/mavlink.h>

namespace Status
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
    static struct tm_repo_t
    {
        mavlink_hm1_tm_t hm1_tm;
        mavlink_ign_tm_t ign_tm;
        mavlink_hr_tm_t hr_tm;
        mavlink_lr_tm_t lr_tm;
        mavlink_logger_tm_t logger_tm;
        mavlink_tmtc_tm_t tmtc_tm;
        mavlink_sm_tm_t sm_tm;
        mavlink_ign_ctrl_tm_t ign_ctrl_tm;
        mavlink_dpl_ctrl_tm_t dpl_ctrl_tm;
        mavlink_ada_tm_t ada_tm;
        mavlink_can_tm_t can_tm;
        mavlink_ad7994_tm_t ad7994_tm;
        mavlink_adc_tm_t adc_tm;
        mavlink_adis_tm_t adis_tm;
        mavlink_mpu_tm_t mpu_tm;
        mavlink_gps_tm_t gps_tm;
        mavlink_pos_tm_t pos_tm;
        mavlink_sm_task1_tm_t sm_task1_tm;
        mavlink_sm_task2_tm_t sm_task2_tm;
        mavlink_sm_task3_tm_t sm_task3_tm;
        mavlink_sm_task4_tm_t sm_task4_tm;
        mavlink_sm_task5_tm_t sm_task5_tm;
    } tm_repository;

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

} /* namespace StatusRepo */