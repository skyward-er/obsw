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

#include <Parafoil/TelemetriesTelecommands/TMRepository.h>

using namespace Boardcore;

namespace Parafoil
{

mavlink_message_t TMRepository::packTM(uint8_t req_tm, uint8_t sys_id,
                                       uint8_t comp_id)
{
    mavlink_message_t m;
    mavlink_nack_tm_t nack_tm;

    miosix::PauseKernelLock kLock;

    switch (req_tm)
    {
        case MavTMList::MAV_SENSORS_TM_ID:
            tm_repository.sensors_tm.timestamp = miosix::getTick();
            mavlink_msg_sensors_tm_encode(sys_id, comp_id, &m,
                                          &(tm_repository.sensors_tm));
            break;
        /*case MavTMList::MAV_SYS_TM_ID:
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
        case MavTMList::MAV_TASK_STATS_TM_ID:
            tm_repository.task_stats_tm.timestamp = miosix::getTick();
            mavlink_msg_task_stats_tm_encode(sys_id, comp_id, &m,
                                            &(tm_repository.task_stats_tm));
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
            //tm_repository.tm_repository.lr_tm.timestamp = miosix::getTick();
            mavlink_msg_lr_tm_encode(sys_id, comp_id, &m,
                                    &(tm_repository.lr_tm));
            break;*/
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

// Implement all the update functions
void TMRepository::update(MPU9250Data data)
{
    // Pause the kernel to avoid interructions during this fast operation
    miosix::PauseKernelLock kLock;
    // Update only the sensors message TODO update all the things
    tm_repository.sensors_tm.bmx160_acc_x = data.accelerationX;
    tm_repository.sensors_tm.bmx160_acc_y = data.accelerationY;
    tm_repository.sensors_tm.bmx160_acc_z = data.accelerationZ;

    tm_repository.sensors_tm.bmx160_gyro_x = data.angularVelocityX;
    tm_repository.sensors_tm.bmx160_gyro_y = data.angularVelocityY;
    tm_repository.sensors_tm.bmx160_gyro_z = data.angularVelocityZ;

    tm_repository.sensors_tm.bmx160_mag_x = data.magneticFieldX;
    tm_repository.sensors_tm.bmx160_mag_y = data.magneticFieldY;
    tm_repository.sensors_tm.bmx160_mag_z = data.magneticFieldZ;

    tm_repository.sensors_tm.bmx160_temp = data.temperature;
}

void TMRepository::update(UbloxGPSData data)
{
    // Pause the kernel to avoid interructions during this fast operation
    miosix::PauseKernelLock kLock;
    // Update only the sensors message TODO update all the things
    tm_repository.sensors_tm.gps_alt = data.height;
    tm_repository.sensors_tm.gps_fix = data.fix;
    tm_repository.sensors_tm.gps_lon = data.longitude;
    tm_repository.sensors_tm.gps_lat = data.latitude;
}

void TMRepository::update(BME280Data data)
{
    // Pause the kernel to avoid interructions during this fast operation
    miosix::PauseKernelLock kLock;
    // Update only the sensors message TODO update all the things
    tm_repository.sensors_tm.ms5803_press = data.pressure;
    tm_repository.sensors_tm.ms5803_temp  = data.temperature;
}

}  // namespace Parafoil
