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
    mavlink_nack_tm_t nack;

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

            tmRepository.loggerTm.buffers_filled    = stats.buffersFilled;
            tmRepository.loggerTm.buffers_written   = stats.buffersWritten;
            tmRepository.loggerTm.dropped_samples   = stats.droppedSamples;
            tmRepository.loggerTm.max_write_time    = stats.maxWriteTime;
            tmRepository.loggerTm.queued_samples    = stats.queuedSamples;
            tmRepository.loggerTm.too_large_samples = stats.tooLargeSamples;
            tmRepository.loggerTm.writes_failed     = stats.writesFailed;
            tmRepository.loggerTm.max_write_time    = stats.maxWriteTime;
            tmRepository.loggerTm.last_write_error  = stats.lastWriteError;

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

            tmRepository.flightTm.nas_n  = state.n;
            tmRepository.flightTm.nas_e  = state.e;
            tmRepository.flightTm.nas_d  = state.d;
            tmRepository.flightTm.nas_vn = state.vn;
            tmRepository.flightTm.nas_ve = state.ve;
            tmRepository.flightTm.nas_vd = state.vd;

            tmRepository.flightTm.nas_qx = state.qx;
            tmRepository.flightTm.nas_qy = state.qy;
            tmRepository.flightTm.nas_qz = state.qz;
            tmRepository.flightTm.nas_qw = state.qw;

            tmRepository.flightTm.nas_bias_x = state.bx;
            tmRepository.flightTm.nas_bias_y = state.by;
            tmRepository.flightTm.nas_bias_z = state.bz;

            mavlink_msg_payload_flight_tm_encode(sys_id, comp_id, &m,
                                                 &(tmRepository.flightTm));

            break;
        }
        default:
        {
            LOG_DEBUG(logger, "Unknown telemetry id: {:d}", req_tm);
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(sys_id, comp_id, &m, &nack);
            break;
        }
    }
    return m;
}

mavlink_message_t TMRepository::packSensorTM(uint8_t req_tm, uint8_t sys_id,
                                             uint8_t comp_id)
{
    mavlink_message_t m;
    mavlink_nack_tm_t nack;

    // The kernel lock is not necessary because the sensors getters already do
    // that

    switch (req_tm)
    {
        case SensorsTMList::MAV_GPS_ID:
        {
            // Get with lock the gps data
            UBXGPSData gps =
                ParafoilTest::getInstance().sensors->getGPSLastSample();

            // Update the repository
            tmRepository.gpsTm.timestamp = miosix::getTick();
            strcpy(tmRepository.gpsTm.sensor_id, "UbloxGPS\0");
            tmRepository.gpsTm.fix          = gps.fix;
            tmRepository.gpsTm.latitude     = gps.latitude;
            tmRepository.gpsTm.longitude    = gps.longitude;
            tmRepository.gpsTm.height       = gps.height;
            tmRepository.gpsTm.vel_north    = gps.velocityNorth;
            tmRepository.gpsTm.vel_east     = gps.velocityEast;
            tmRepository.gpsTm.vel_down     = gps.velocityDown;
            tmRepository.gpsTm.speed        = gps.speed;
            tmRepository.gpsTm.track        = gps.track;
            tmRepository.gpsTm.n_satellites = gps.satellites;

            // Encode the message
            mavlink_msg_gps_tm_encode(sys_id, comp_id, &m,
                                      &(tmRepository.gpsTm));
            break;
        }
        case SensorsTMList::MAV_MPU9250_ID:
        {
            // Get with lock the imu data
            MPU9250Data imu =
                ParafoilTest::getInstance().sensors->getMPU9250LastSample();

            // Update the repository
            tmRepository.imuTm.timestamp = miosix::getTick();
            strcpy(tmRepository.imuTm.sensor_id, "MPU9250\0");
            tmRepository.imuTm.acc_x  = imu.accelerationX;
            tmRepository.imuTm.acc_y  = imu.accelerationY;
            tmRepository.imuTm.acc_z  = imu.accelerationZ;
            tmRepository.imuTm.gyro_x = imu.angularVelocityX;
            tmRepository.imuTm.gyro_y = imu.angularVelocityY;
            tmRepository.imuTm.gyro_z = imu.angularVelocityZ;
            tmRepository.imuTm.mag_x  = imu.magneticFieldX;
            tmRepository.imuTm.mag_y  = imu.magneticFieldY;
            tmRepository.imuTm.mag_z  = imu.magneticFieldZ;

            // Encode the message
            mavlink_msg_imu_tm_encode(sys_id, comp_id, &m,
                                      &(tmRepository.imuTm));
            break;
        }
        case SensorsTMList::MAV_BME280_ID:
        {
            // Get with lock the barometer data
            BME280Data baro =
                ParafoilTest::getInstance().sensors->getBME280LastSample();

            // Update the repository
            tmRepository.barometerTm.timestamp = miosix::getTick();
            strcpy(tmRepository.barometerTm.sensor_id, "BME280\0");
            tmRepository.barometerTm.pressure = baro.pressure;

            // Encode the message
            mavlink_msg_baro_tm_encode(sys_id, comp_id, &m,
                                       &(tmRepository.barometerTm));
            break;
        }
        default:
        {
            // Send the nack
            LOG_DEBUG(logger, "Unknown telemetry id: {:d}", req_tm);
            nack.recv_msgid = 0;
            nack.seq_ack    = 0;
            mavlink_msg_nack_tm_encode(sys_id, comp_id, &m, &nack);
        }
    }

    return m;
}

}  // namespace Parafoil
