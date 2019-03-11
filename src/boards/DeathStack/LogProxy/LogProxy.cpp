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

using namespace Status;

namespace DeathStackBoard 
{

template <>
LogResult LoggerProxy::log<FMMStatus>(const FMMStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare pin_last_detection e pin_detection_count
    tm_repository.hm1_tm.state = t.state;


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
    tm_repository.ign_ctrl_tm.n_rcv_messagen_rcv_message = t.n_rcv_messages;
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
LogResult LoggerProxy::log<MPU9250Data>(const MPU9250Data& t)
{
    miosix::PauseKernelLock kLock;

    // TODO aggiornare la status repo

    tm_repository.mpu_tm.acc_x = t.accel.getX();
    tm_repository.mpu_tm.acc_y = t.accel.getY();
    tm_repository.mpu_tm.acc_z = t.accel.getZ();
   
    return logger.log(t);
}

}