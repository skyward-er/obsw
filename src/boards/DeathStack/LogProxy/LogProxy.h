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

#pragma once

#include "FlightStats.h"
#include "Singleton.h"
#include "logger/Logger.h"

#include "FlightStats.h"
#include "Telemetries.h"

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack/DeathStackStatus.h"
#include "DeathStack/DeploymentController/DeploymentData.h"
#include "DeathStack/FlightModeManager/FMMStatus.h"
#include "DeathStack/IgnitionController/IgnitionStatus.h"
#include "DeathStack/PinObserver/PinObserverData.h"
#include "DeathStack/SensorManager/SensorManagerData.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapperData.h"
#include "DeathStack/SensorManager/Sensors/PiksiData.h"

#include "drivers/canbus/CanUtils.h"
#include "drivers/mavlink/MavStatus.h"
#include "scheduler/TaskSchedulerData.h"
#include "sensors/ADIS16405/ADIS16405Data.h"
#include "sensors/MPU9250/MPU9250Data.h"

namespace DeathStackBoard
{

/**
 * @brief This class is interposed between the OBSW and the Logger driver.
 * Status repository updating is done here: everytime a component
 * logs its status, the corresponding tm structs are updated before logging
 * on SD card.
 */
class LoggerProxy : public Singleton<LoggerProxy>
{
    friend class Singleton<LoggerProxy>;

public:
    LoggerProxy() : logger(Logger::instance()) { initTelemetries(); }

    ~LoggerProxy() {}

    /* Generic log function, to be implemented for each loggable struct */
    template <typename T>
    inline LogResult log(const T& t)
    {
        return logger.log(t);
    }

    /**
     * Blocking call. May take a long time.
     *
     * Call this function to start the logger.
     * When this function returns, the logger is started, and subsequent calls
     * to log will actually log the data.
     *
     * \throws runtime_error if the log could not be opened
     * \return log number
     */
    int start() { return logger.start(); }

    /**
     * Blocking call. May take a very long time (seconds).
     *
     * Call this function to stop the logger.
     * When this function returns, all log buffers have been flushed to disk,
     * and it is safe to power down the board without losing log data or
     * corrupting the filesystem.
     */
    void stop() { logger.stop(); }

private:
    Logger& logger;  // SD logger
    FlightStats flight_stats{};
};

template <>
LogResult LoggerProxy::log<DeathStackStatus>(const DeathStackStatus& t);

/* Flight Mode Manager */
template <>
LogResult LoggerProxy::log<FMMStatus>(const FMMStatus& t);

/* Launch and Nosecone detachment pins */
template <>
LogResult LoggerProxy::log<PinStatus>(const PinStatus& t);

/* Ignition Board */
template <>
LogResult LoggerProxy::log<IgnBoardLoggableStatus>(
    const IgnBoardLoggableStatus& t);

/* Ignition Controller */
template <>
LogResult LoggerProxy::log<IgnCtrlStatus>(const IgnCtrlStatus& t);

/* Logger */
template <>
LogResult LoggerProxy::log<LogStats>(const LogStats& t);

/* TMTCManager (Mavlink) */
template <>
LogResult LoggerProxy::log<MavStatus>(const MavStatus& t);

/* Sensor Manager */
template <>
LogResult LoggerProxy::log<SensorManagerStatus>(const SensorManagerStatus& t);

/* Deployment Controller */
template <>
LogResult LoggerProxy::log<DeploymentStatus>(const DeploymentStatus& t);

/* ADA state machine */
template <>
LogResult LoggerProxy::log<ADAStatus>(const ADAStatus& t);

/* ADA target dpl pressure */
template <>
LogResult LoggerProxy::log<TargetDeploymentAltitude>(
    const TargetDeploymentAltitude& t);

/* ADA kalman filter values */
template <>
LogResult LoggerProxy::log<KalmanState>(const KalmanState& t);

/* ADA kalman altitude values */
template <>
LogResult LoggerProxy::log<KalmanAltitude>(const KalmanAltitude& t);

template <>
LogResult LoggerProxy::log<ReferenceValues>(const ReferenceValues& t);

template <>
LogResult LoggerProxy::log<ADACalibrationData>(const ADACalibrationData& t);

/* Canbus stats */
template <>
LogResult LoggerProxy::log<CanStatus>(const CanStatus& t);

/* Main Barometer */
template <>
LogResult LoggerProxy::log<AD7994WrapperData>(const AD7994WrapperData& t);

/* Battery status, sampled by internal ADC */
template <>
LogResult LoggerProxy::log<BatteryVoltageData>(const BatteryVoltageData& t);

/* Motor current sense, sampled by internal ADC */
template <>
LogResult LoggerProxy::log<CurrentSenseData>(const CurrentSenseData& t);

/* ADIS imu */
template <>
LogResult LoggerProxy::log<ADIS16405Data>(const ADIS16405Data& t);

/* MPU imu */
template <>
LogResult LoggerProxy::log<MPU9250Data>(const MPU9250Data& t);

/* GPS */
template <>
LogResult LoggerProxy::log<PiksiData>(const PiksiData& t);

/* LM75b temperature */
template <>
LogResult LoggerProxy::log<LM75BData>(const LM75BData& t);

template <>
LogResult LoggerProxy::log<TaskStatResult>(const TaskStatResult& t);

template <>
LogResult LoggerProxy::log<LiftOffStats>(const LiftOffStats& t);

template <>
LogResult LoggerProxy::log<ApogeeStats>(const ApogeeStats& t);

template <>
LogResult LoggerProxy::log<DrogueDPLStats>(const DrogueDPLStats& t);

template <>
LogResult LoggerProxy::log<MainDPLStats>(const MainDPLStats& t);

}  // namespace DeathStackBoard