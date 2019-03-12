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

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack/DeploymentController/DeploymentData.h"
#include "DeathStack/FlightModeManager/FMMStatus.h"
#include "DeathStack/IgnitionController/IgnitionStatus.h"
#include "DeathStack/PinObserver/PinObserverData.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapperData.h"
#include "DeathStack/SensorManager/SensorManagerData.h"

#include "skyward-boardcore/src/shared/drivers/canbus/CanUtils.h"
#include "skyward-boardcore/src/shared/drivers/mavlink/MavStatus.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include "sensors/ADIS16405/ADIS16405Data.h"
#include "scheduler/TaskSchedulerData.h"
#include "drivers/piksi/piksi_data.h"

using namespace Status;

namespace DeathStackBoard 
{

template <>
LogResult LoggerProxy::log<FMMStatus>(const FMMStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare pin_last_detection e pin_detection_count
    tm_repository.hm1_tm.state = static_cast<uint8_t>(t.state);


    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<IgnBoardLoggableStatus>(const IgnBoardLoggableStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<LogStats>(const LogStats& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<MavStatus>(const MavStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<SensorManagerStatus>(const SensorManagerStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<IgnCtrlStatus>(const IgnCtrlStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
    //manca abort_rcv, abort_sent

    tm_repository.ign_ctrl_tm.timestamp = t.timestamp;
    tm_repository.ign_ctrl_tm.fsm_state = t.fsm_state;
    tm_repository.ign_ctrl_tm.last_event = t.last_event;
    tm_repository.ign_ctrl_tm.n_rcv_message= t.n_rcv_messages;
    tm_repository.ign_ctrl_tm.n_sent_messages = t.launch_sent;
    //tm_repository.ign_ctrl_tm.abort = t.abort_rcv;
    //tm_repository.ign_ctrl_tm. = t.abort_sent;
    
    
    
    t.n_sent_messages;
    t.padding;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<DeploymentStatus>(const DeploymentStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<ADAStatus>(const ADAStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<KalmanState>(const KalmanState& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<CanStatus>(const CanStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<AD7994WrapperData>(const AD7994WrapperData& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<BatteryTensionData>(const BatteryTensionData& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<CurrentSenseData>(const CurrentSenseData& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo

    return logger.log(t);
}

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

    // GPS_TM
    tm_repository.gps_tm.timestamp = t.timestamp;
    tm_repository.gps_tm.lat = t.latitude;
    tm_repository.gps_tm.lon = t.longitude;
    tm_repository.gps_tm.altitude = t.height;
    tm_repository.gps_tm.vel_north = t.velocityNorth;
    tm_repository.gps_tm.vel_east = t.velocityEast;
    tm_repository.gps_tm.vel_down = t.velocityDown;
    tm_repository.gps_tm.vel_mag = t.speed;

    tm_repository.gps_tm.fix = (uint8_t) t.fix;
    tm_repository.gps_tm.n_satellites = t.numSatellites;

    // LR_TM
    tm_repository.lr_tm.gps_alt = t.height;
    tm_repository.lr_tm.gps_vel_mag = t.speed;

    // POS_TM
    tm_repository.pos_tm.lat = t.latitude;
    tm_repository.pos_tm.lon = t.longitude;
   
    return logger.log(t);
}

// Sorry but it's much clearer without repeated code
#define UPDATE_TASK(n) \
			tm_repository.sm_task##n##_tm.task_##n##_id = t.id; \
    		tm_repository.sm_task##n##_tm.task_##n##_min_value = t.activationStats.minValue; \
    		tm_repository.sm_task##n##_tm.task_##n##_max_value = t.activationStats.maxValue; \
    		tm_repository.sm_task##n##_tm.task_##n##_mean_value = t.activationStats.mean; \
    		tm_repository.sm_task##n##_tm.task_##n##_stddev = t.activationStats.stdev;

template <>
LogResult LoggerProxy::log<TaskStatResult>(const TaskStatResult& t)
{
    miosix::PauseKernelLock kLock;

    switch (t.id)
    {
    	case 1:
    		UPDATE_TASK(1);
    		break;
    	case 2:
    		UPDATE_TASK(2);
    		break;
    	case 3:
    		UPDATE_TASK(4);
    		break;
    	case 4:
    		UPDATE_TASK(4);
    		break;
    	case 5:
    		UPDATE_TASK(5);
    		break;
    	default:
    		break;
    }
   
    return logger.log(t);
}

}