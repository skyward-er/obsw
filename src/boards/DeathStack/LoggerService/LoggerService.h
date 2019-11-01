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

#include "Singleton.h"
#include "logger/Logger.h"

#include "FlightStatsRecorder.h"
#include "TmRepository.h"

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack/DeathStackStatus.h"
#include "DeathStack/DeploymentController/DeploymentData.h"
#include "DeathStack/FlightModeManager/FMMStatus.h"
#include "DeathStack/IgnitionController/IgnitionStatus.h"
#include "DeathStack/PinHandler/PinHandlerData.h"
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
 * Every time a component logs its status, the log function for that specific
 * struct is called. In this way, the Logger updates the Tm Repository before
 * logging on SD card.
 */
class LoggerService : public Singleton<LoggerService>
{
    friend class Singleton<LoggerService>;

public:
    LoggerService() : logger(Logger::instance())
    {
        initTelemetries();
        flight_stats.start();
    }

    ~LoggerService() {}

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
    FlightStatsRecorder flight_stats{};
};

template <>
LogResult LoggerService::log<DeathStackStatus>(const DeathStackStatus& t);

/* Flight Mode Manager */
template <>
LogResult LoggerService::log<FMMStatus>(const FMMStatus& t);

/* Launch and Nosecone detachment pins */
template <>
LogResult LoggerService::log<PinStatus>(const PinStatus& t);

/* Ignition Board */
template <>
LogResult LoggerService::log<IgnBoardLoggableStatus>(
    const IgnBoardLoggableStatus& t);

/* Ignition Controller */
template <>
LogResult LoggerService::log<IgnCtrlStatus>(const IgnCtrlStatus& t);

/* Logger */
template <>
LogResult LoggerService::log<LogStats>(const LogStats& t);

/* TMTCManager (Mavlink) */
template <>
LogResult LoggerService::log<MavStatus>(const MavStatus& t);

/* Sensor Manager */
template <>
LogResult LoggerService::log<SensorManagerStatus>(const SensorManagerStatus& t);

/* Deployment Controller */
template <>
LogResult LoggerService::log<DeploymentStatus>(const DeploymentStatus& t);

/* ADA state machine */
template <>
LogResult LoggerService::log<ADAStatus>(const ADAStatus& t);

/* ADA target dpl pressure */
template <>
LogResult LoggerService::log<TargetDeploymentAltitude>(
    const TargetDeploymentAltitude& t);

/* ADA kalman filter values */
template <>
LogResult LoggerService::log<KalmanState>(const KalmanState& t);

/* ADA kalman altitude values */
template <>
LogResult LoggerService::log<KalmanAltitude>(const KalmanAltitude& t);

template <>
LogResult LoggerService::log<ReferenceValues>(const ReferenceValues& t);

/* Canbus stats */
template <>
LogResult LoggerService::log<CanStatus>(const CanStatus& t);

/* Main Barometer */
template <>
LogResult LoggerService::log<AD7994WrapperData>(const AD7994WrapperData& t);

/* Battery status, sampled by internal ADC */
template <>
LogResult LoggerService::log<BatteryVoltageData>(const BatteryVoltageData& t);

/* Motor current sense, sampled by internal ADC */
template <>
LogResult LoggerService::log<CurrentSenseData>(const CurrentSenseData& t);

/* ADIS imu */
template <>
LogResult LoggerService::log<ADIS16405Data>(const ADIS16405Data& t);

/* MPU imu */
template <>
LogResult LoggerService::log<MPU9250Data>(const MPU9250Data& t);

/* GPS */
template <>
LogResult LoggerService::log<PiksiData>(const PiksiData& t);

/* LM75b temperature */
template <>
LogResult LoggerService::log<LM75BData>(const LM75BData& t);

template <>
LogResult LoggerService::log<TaskStatResult>(const TaskStatResult& t);

template <>
LogResult LoggerService::log<LiftOffStats>(const LiftOffStats& t);

template <>
LogResult LoggerService::log<ApogeeStats>(const ApogeeStats& t);

template <>
LogResult LoggerService::log<DrogueDPLStats>(const DrogueDPLStats& t);

template <>
LogResult LoggerService::log<MainDPLStats>(const MainDPLStats& t);

}  // namespace DeathStackBoard