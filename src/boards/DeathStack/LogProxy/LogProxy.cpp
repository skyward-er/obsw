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

    // TODO sistemare il bitfield

    tm_repository.ign_tm.timestamp = t.timestamp;
    //tm_repository.stm32_bitfield = t.board_status.avr_abortCmd;
    //tm_repository.stm32_bitfield = t.board_status.avr_abortTimeout;
    //tm_repository.stm32_bitfield = t.board_status.avr_abortWrongCode;
    //tm_repository.stm32_bitfield = t.board_status.avr_launchDone;
    //tm_repository.stm32_bitfield = t.board_status.stm32_powerOnReset;
    //tm_repository.stm32_bitfield = t.board_status.stm32_externalReset;
    //tm_repository.stm32_bitfield = t.board_status.stm32_brownoutReset;
    //tm_repository.stm32_bitfield = t.board_status.stm32_watchdogReset;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<LogStats>(const LogStats& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.logger_tm.timestamp = t.timestamp;
    tm_repository.logger_tm.statTooLargeSamples = t.statTooLargeSamples;
    tm_repository.logger_tm.statDroppedSamples = t.statDroppedSamples;
    tm_repository.logger_tm.statQueuedSamples = t.statQueuedSamples;
    tm_repository.logger_tm.statBufferFilled = t.statBufferFilled;    
    tm_repository.logger_tm.statBufferWritten = t.statBufferWritten;
    tm_repository.logger_tm.statWriteFailed = t.statWriteFailed;
    tm_repository.logger_tm.statWriteError = t.statWriteError;
    tm_repository.logger_tm.statWriteTime = t.statWriteTime;
    tm_repository.logger_tm.statMaxWriteTime = t.statMaxWriteTime;

    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<MavStatus>(const MavStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO sistemare mav_stats
    // TODO controllare se è normale che ci siano solo questi elementi nella struct MavStatus
    // TODO controllare tmtc_tm, alcuni nomi non corrispondono con la specifica Hermes Status
    tm_repository.tmtc_tm.timestamp = t.timestamp;
    tm_repository.tmtc_tm.n_send_queue = t.n_send_queue;
    tm_repository.tmtc_tm.max_send_queue = t.max_send_queue;
    tm_repository.tmtc_tm.n_send_errors = t.n_send_errors;
    //t.mav_stats;
   
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

    // TODO in ign_ctrl_tm, è da aggiungere una s finale a n_rcv_message per essere conforme con la specifica
    // TODO manca cmd_bitfield, dove bisogna mettere abort_rcv, abort_sent, launch_sent e padding
    // TODO inserire nel commento di cmd_bitfield di mavlink_ign_ctrl_tm_t anche la presenza di padding

    tm_repository.ign_ctrl_tm.timestamp = t.timestamp;
    tm_repository.ign_ctrl_tm.fsm_state = t.fsm_state;
    tm_repository.ign_ctrl_tm.last_event = t.last_event;
    tm_repository.ign_ctrl_tm.n_rcv_message = t.n_rcv_messages;
    tm_repository.ign_ctrl_tm.n_sent_messages = t.n_sent_messages;
    //tm_repository.ign_ctrl_tm.cmd_bitfield = t.launch_sent;
    //tm_repository.ign_ctrl_tm.cmd_bitfield = t.abort_rcv;
    //tm_repository.ign_ctrl_tm.cmd_bitfield = t.abort_sent;
    //tm_repository.ign_ctrl_tm.cmd_bitfield = t.padding;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<DeploymentStatus>(const DeploymentStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO sistemare il cast dei tipi motor_last_direction e cutter_state
    tm_repository.dpl_ctrl_tm.timestamp = t.timestamp;
    tm_repository.dpl_ctrl_tm.fsm_state = t.state;
    tm_repository.dpl_ctrl_tm.motor_active = t.motor_status.motor_active;
    tm_repository.dpl_ctrl_tm.motor_last_direction = t.motor_status.motor_last_direction;
    tm_repository.dpl_ctrl_tm.cutter_state = t.cutter_status.state;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<ADAStatus>(const ADAStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO sistema cast dei tipi stae e last_apogee_state
    // TODO target_dpl_pressure non è presente. Controllare se è da aggiungere
    // TO DO last_dpl_pressure_tick ha un nome diverso rispetto alle speciiche
    tm_repository.ada_tm.timestamp = t.timestamp;
    tm_repository.ada_tm.state = t.state;
    tm_repository.ada_tm.last_apogee_state = t.last_apogee.state;
    tm_repository.ada_tm.last_apogee_tick = t.last_apogee.tick;
    tm_repository.ada_tm.last_dpl_pressure_tick = t.last_dpl_pressure_tick;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<KalmanState>(const KalmanState& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.ada_tm.kalman_x0 = t.x0;
    tm_repository.ada_tm.kalman_x1 = t.x1;
    tm_repository.ada_tm.kalman_x2 = t.x2;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<CanStatus>(const CanStatus& t)
{
    miosix::PauseKernelLock kLock;

    // TODO nomi non corrispondono a specifica
    tm_repository.can_tm.n_sent = t.n_sent;
    tm_repository.can_tm.n_rcv = t.n_rcv;
    tm_repository.can_tm.last_sent = t.last_sent;
    tm_repository.can_tm.last_rcv = t.last_rcv;
    tm_repository.can_tm.last_sent_ts = t.last_sent_ts;
    tm_repository.can_tm.last_rcv_ts = t.last_rcv_ts;

    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<AD7994WrapperData>(const AD7994WrapperData& t)
{
    miosix::PauseKernelLock kLock;

    //nomi non corrispondono a specifiche
    // TODO manca val_ch1_min
    // TO DO la classe AD7994WrapperData ha dei valori in più, come i flag
    tm_repository.ad7994_tm.timestamp = t.timestamp;
    tm_repository.ad7994_tm.val_ch1 = t.ch1_pressure;
    //t.val_ch1_min
    tm_repository.ad7994_tm.val_ch2 = t.ch2_pressure;
    tm_repository.ad7994_tm.mean_ch1 = t.ch1_mean;
    tm_repository.ad7994_tm.stddev_ch1 = t.ch1_stddev;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<BatteryTensionData>(const BatteryTensionData& t)
{
    miosix::PauseKernelLock kLock;

    tm_repository.adc_tm.timestamp = t.timestamp;
    tm_repository.adc_tm.battery_tension = t.battery_tension_value;
    tm_repository.adc_tm.battery_tension_min = t.battery_tension_min;
   
    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<CurrentSenseData>(const CurrentSenseData& t)
{
    miosix::PauseKernelLock kLock;

    //TO DO anche questo aggiorna timestamp, oltre a BatteryTensionData. Controllare.
    tm_repository.adc_tm.timestamp = t.timestamp;
    tm_repository.adc_tm.current_sense_1 = t.current_1_value;
    tm_repository.adc_tm.current_sense_2 = t.current_2_value;
   
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