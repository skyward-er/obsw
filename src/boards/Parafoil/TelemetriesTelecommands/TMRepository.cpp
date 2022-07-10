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

#include <Parafoil/ParafoilTest.h>
#include <Parafoil/TelemetriesTelecommands/TMRepository.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <Eigen/Core>

using namespace Boardcore;
using namespace Eigen;

namespace Parafoil
{

mavlink_message_t TMRepository::packSystemTM(uint8_t req_tm, uint8_t sys_id,
                                             uint8_t comp_id)
{
    mavlink_message_t m;
    mavlink_nack_tm_t nack_tm;

    miosix::PauseKernelLock kLock;

    switch (req_tm)
    {
        case SystemTMList::MAV_LOGGER_ID:
        {
            tmRepository.loggerTm.timestamp = miosix::getTick();

            // Get the logger stats
            Boardcore::LoggerStats stats =
                ParafoilTest::getInstance().SDlogger->getStats();

            // First i update the loggerTm
            tmRepository.loggerTm.log_number =
                ParafoilTest::getInstance().SDlogger->getCurrentLogNumber();

            tmRepository.loggerTm.filled_buffers    = stats.buffersFilled;
            tmRepository.loggerTm.written_buffers   = stats.buffersWritten;
            tmRepository.loggerTm.sdropped_samples  = stats.droppedSamples;
            tmRepository.loggerTm.max_write_time    = stats.maxWriteTime;
            tmRepository.loggerTm.queued_samples    = stats.queuedSamples;
            tmRepository.loggerTm.too_large_samples = stats.tooLargeSamples;
            tmRepository.loggerTm.failed_writes     = stats.writesFailed;
            tmRepository.loggerTm.max_write_time    = stats.maxWriteTime;
            tmRepository.loggerTm.error_writes      = stats.lastWriteError;

            mavlink_msg_logger_tm_encode(sys_id, comp_id, &m,
                                         &(tmRepository.loggerTm));
            break;
        }
        case SystemTMList::MAV_FLIGHT_ID:
        {
            // I have to send the whole flight tm
            tmRepository.flightTm.timestamp = miosix::getTick();

            // Get the pressure
            tmRepository.flightTm.pressure_digi =
                ParafoilTest::getInstance()
                    .sensors->getBME280LastSample()
                    .pressure;

            // Get the IMU data
            MPU9250Data imu =
                ParafoilTest::getInstance().sensors->getMPU9250LastSample();

            tmRepository.flightTm.acc_x  = imu.accelerationX;
            tmRepository.flightTm.acc_y  = imu.accelerationY;
            tmRepository.flightTm.acc_z  = imu.accelerationZ;
            tmRepository.flightTm.gyro_x = imu.angularVelocityX;
            tmRepository.flightTm.gyro_y = imu.angularVelocityY;
            tmRepository.flightTm.gyro_z = imu.angularVelocityZ;
            tmRepository.flightTm.mag_x  = imu.magneticFieldX;
            tmRepository.flightTm.mag_y  = imu.magneticFieldY;
            tmRepository.flightTm.mag_z  = imu.magneticFieldZ;

            // Get the GPS data
            UBXGPSData gps =
                ParafoilTest::getInstance().sensors->getGPSLastSample();

            tmRepository.flightTm.gps_fix = gps.fix;
            tmRepository.flightTm.gps_lat = gps.latitude;
            tmRepository.flightTm.gps_lon = gps.longitude;
            tmRepository.flightTm.gps_alt = gps.height;

            // Get the NAS data
            NASState state =
                ParafoilTest::getInstance().algorithms->getNASLastSample();

            tmRepository.flightTm.nas_x  = state.n;
            tmRepository.flightTm.nas_y  = state.e;
            tmRepository.flightTm.nas_z  = state.d;
            tmRepository.flightTm.nas_vx = state.vn;
            tmRepository.flightTm.nas_vy = state.ve;
            tmRepository.flightTm.nas_vz = state.vd;

            // TODO discuss about quaternion to euler computation in this
            // instance

            tmRepository.flightTm.nas_bias0 = state.bx;
            tmRepository.flightTm.nas_bias1 = state.by;
            tmRepository.flightTm.nas_bias2 = state.bz;

            break;
        }
        /*case MavTMList::MAV_SYS_TM_ID:
            tmRepository.sys_tm.timestamp = miosix::getTick();
            mavlink_msg_sys_tm_encode(sys_id, comp_id, &m,
                                    &(tmRepository.sys_tm));
            break;
        case MavTMList::MAV_FMM_TM_ID:
            tmRepository.fmm_tm.timestamp = miosix::getTick();
            mavlink_msg_fmm_tm_encode(sys_id, comp_id, &m,
                                    &(tmRepository.fmm_tm));
            break;
        case MavTMList::MAV_TMTC_TM_ID:
            tmRepository.tmtc_tm.timestamp = miosix::getTick();
            mavlink_msg_tmtc_tm_encode(sys_id, comp_id, &m,
                                    &(tmRepository.tmtc_tm));
            break;
        case MavTMList::MAV_TASK_STATS_TM_ID:
            tmRepository.task_stats_tm.timestamp = miosix::getTick();
            mavlink_msg_task_stats_tm_encode(sys_id, comp_id, &m,
                                            &(tmRepository.task_stats_tm));
            break;
        case MavTMList::MAV_GPS_TM_ID:
            tmRepository.gps_tm.timestamp = miosix::getTick();
            mavlink_msg_gps_tm_encode(sys_id, comp_id, &m,
                                    &(tmRepository.gps_tm));
            break;
        case MavTMList::MAV_HR_TM_ID:
            tmRepository.hr_tm.timestamp = miosix::getTick();
            mavlink_msg_hr_tm_encode(sys_id, comp_id, &m,
                                    &(tmRepository.hr_tm));
            break;
        case MavTMList::MAV_LR_TM_ID:
            //tmRepository.tmRepository.lr_tm.timestamp = miosix::getTick();
            mavlink_msg_lr_tm_encode(sys_id, comp_id, &m,
                                    &(tmRepository.lr_tm));
            break;*/
        default:
        {
            LOG_DEBUG(logger, "Unknown telemetry id: {:d}", req_tm);
            nack_tm.recv_msgid = 0;
            nack_tm.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(sys_id, comp_id, &m, &nack_tm);
            break;
        }
    }
    return m;
}

mavlink_message_t TMRepository::packSensorTM(uint8_t req_tm, uint8_t sys_id,
                                             uint8_t comp_id)
{
    mavlink_message_t m;
    mavlink_nack_tm_t nack_tm;

    miosix::PauseKernelLock kLock;

    return m;
}

}  // namespace Parafoil
