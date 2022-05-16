/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "TMRepository.h"

namespace Main
{

mavlink_message_t TMRepository::packSystemTm(SystemTMList reqTm)
{
    mavlink_message_t msg;

    // Prevent preemption, MUST not yeld or use the kernel!
    miosix::PauseKernelLock kLock;

    switch (reqTm)
    {
        // System telemetries
        case SystemTMList::MAV_SYS_ID:
            break;
        case SystemTMList::MAV_FSM_ID:
            break;
        case SystemTMList::MAV_PIN_OBS_ID:
            break;
        case SystemTMList::MAV_LOGGER_ID:
            break;
        case SystemTMList::MAV_MAVLINK_STATS:
            break;
        case SystemTMList::MAV_TASK_STATS_ID:
            break;
        case SystemTMList::MAV_DPL_ID:
            break;
        case SystemTMList::MAV_ADA_ID:
            break;
        case SystemTMList::MAV_NAS_ID:
            break;
        case SystemTMList::MAV_CAN_ID:
            break;
        case SystemTMList::MAV_FLIGHT_ID:
            break;
        case SystemTMList::MAV_FLIGHT_STATS_ID:
            break;
        case SystemTMList::MAV_SENSORS_STATE_ID:
            break;

        default:
            LOG_DEBUG(logger, "Unknown telemetry id: {}", reqTm);
            mavlink_nack_tm_t nack;
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(RadioConfigs::MAV_SYSTEM_ID,
                                       RadioConfigs::MAV_COMPONENT_ID, &msg,
                                       &nack);
            break;
    }

    return msg;
}

mavlink_message_t TMRepository::packSensorsTm(SensorsTMList reqTm)
{
    mavlink_message_t msg;

    // Prevent preemption, MUST not yeld or use the kernel!
    miosix::PauseKernelLock kLock;

    switch (reqTm)
    {
        // Sensors telemetries
        case SensorsTMList::MAV_GPS_ID:
            break;
        case SensorsTMList::MAV_BMX160_ID:
            break;
        case SensorsTMList::MAV_VN100_ID:
            break;
        case SensorsTMList::MAV_MPU9250_ID:
            break;
        case SensorsTMList::MAV_CURRENT_SENSE_ID:
            break;
        case SensorsTMList::MAV_LIS3MDL_ID:
            break;
        case SensorsTMList::MAV_DPL_PRESS_ID:
            break;
        case SensorsTMList::MAV_STATIC_PRESS_ID:
            break;
        case SensorsTMList::MAV_PITOT_PRESS_ID:
            break;
        case SensorsTMList::MAV_BATTERY_VOLTAGE_ID:
            break;
        case SensorsTMList::MAV_STRAIN_GAUGE_ID:
            break;

        default:
            LOG_DEBUG(logger, "Unknown telemetry id: {}", reqTm);
            mavlink_nack_tm_t nack;
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(RadioConfigs::MAV_SYSTEM_ID,
                                       RadioConfigs::MAV_COMPONENT_ID, &msg,
                                       &nack);
            break;
    }

    return msg;
}

}  // namespace Main
