/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "LogProxy.h"
#include "DeathStack/Status.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include "sensors/ADIS16405/ADIS16405Data.h"

using namespace Status;

namespace DeathStackBoard 
{

template <>
LogResult LoggerProxy::log<ADIS16405Data>(const ADIS16405Data& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.adis_tm.timestamp = miosix::getTick();

    tm_repository.adis_tm.acc_x = t.xaccl_out;
    tm_repository.adis_tm.acc_y = t.yaccl_out;
    tm_repository.adis_tm.acc_z = t.zaccl_out;

    tm_repository.adis_tm.gyro_x = t.xgyro_out;
    tm_repository.adis_tm.gyro_y = t.ygyro_out;
    tm_repository.adis_tm.gyro_z = t.zgyro_out;

    tm_repository.adis_tm.compass_x = t.xmagn_out;
    tm_repository.adis_tm.compass_y = t.ymagn_out;
    tm_repository.adis_tm.compass_z = t.zmagn_out;

    tm_repository.adis_tm.temp = t.temp_out;
    tm_repository.adis_tm.supply_out = t.supply_out;
    tm_repository.adis_tm.aux_adc = t.aux_adc;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<MPU9250Data>(const MPU9250Data& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.mpu_tm.timestamp = miosix::getTick();

    tm_repository.mpu_tm.acc_x = t.accel.getX();
    tm_repository.mpu_tm.acc_y = t.accel.getY();
    tm_repository.mpu_tm.acc_z = t.accel.getZ();

    tm_repository.mpu_tm.gyro_x = t.gyro.getX();
    tm_repository.mpu_tm.gyro_y = t.gyro.getY();
    tm_repository.mpu_tm.gyro_z = t.gyro.getZ();

    tm_repository.mpu_tm.compass_x = t.compass.getX();
    tm_repository.mpu_tm.compass_y = t.compass.getY();
    tm_repository.mpu_tm.compass_z = t.compass.getZ();

    tm_repository.mpu_tm.temp = t.temp;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<GPSData>(const GPSData& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.gps_tm.timestamp = t.timestamp;

    tm_repository.gps_tm.lat = t.latitude;
    tm_repository.gps_tm.lon = t.logitude;
    tm_repository.gps_tm.altitude = t.height;

    tm_repository.gps_tm.vel_north = t.velocityNorth;
    tm_repository.gps_tm.vel_east = t.velocityEast;
    tm_repository.gps_tm.vel_down = t.velocityDown;
    tm_repository.gps_tm.vel_mag = t.speed;

    tm_repository.gps_tm.fix = (uint8_t) t.fix;
    tm_repository.gps_tm.n_satellites = t.numSatellites;
   
    return logger.log(t);
}

}