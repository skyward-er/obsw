/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <TelemetriesTelecommands/TMRepository.h>

namespace ParafoilTestDev
{
    mavlink_message_t TMRepository::packTM(uint8_t req_tm, uint8_t sys_id, uint8_t comp_id)
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
            case MavTMList::MAV_ADC_TM_ID:
                tm_repository.adc_tm.timestamp = miosix::getTick();
                mavlink_msg_adc_tm_encode(sys_id, comp_id, &m,
                                        &(tm_repository.adc_tm));
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
            case MavTMList::MAV_SENSORS_TM_ID:
                tm_repository.sensors_tm.timestamp = miosix::getTick();
                mavlink_msg_sensors_tm_encode(sys_id, comp_id, &m,
                                            &(tm_repository.sensors_tm));
                break;
            default:
            {
                LOG_DEBUG(logger, "Unknown telemetry id: %d", req_tm);
                nack_tm.recv_msgid = 0;
                nack_tm.seq_ack    = 0;
                mavlink_msg_nack_tm_encode(sys_id, comp_id, &m, &nack_tm);
                break;
            }
        }
        return m;
    }
}