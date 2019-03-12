/** @file
 *  @brief Skyward Status Repository
 */
#pragma once

#include <mavlink_skyward_lib/mavlink_lib/hermes/mavlink.h>

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

} /* namespace StatusRepo */