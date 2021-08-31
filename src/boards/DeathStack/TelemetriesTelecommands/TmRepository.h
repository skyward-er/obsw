/* Copyright (c) 2019-2020 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#pragma once

#include <AirBrakes/AirBrakesData.h>
#include <ApogeeDetectionAlgorithm/ADAData.h>
#include <DeathStackStatus.h>
#include <Deployment/DeploymentData.h>
#include <FlightModeManager/FMMStatus.h>
#include <FlightStatsRecorder/FSRController.h>
#include <FlightStatsRecorder/FSRData.h>
#include <Main/SensorsData.h>
#include <NavigationAttitudeSystem/NASData.h>
#include <PinHandler/PinHandlerData.h>
#include <Singleton.h>
#include <System/SystemData.h>
#include <TelemetriesTelecommands/Mavlink.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/Xbee/APIFramesLog.h>
#include <drivers/adc/ADS1118/ADS1118Data.h>
#include <drivers/gps/ublox/UbloxGPSData.h>
#include <scheduler/TaskSchedulerData.h>
#include <sensors/BMX160/BMX160WithCorrectionData.h>
#include <sensors/LIS3MDL/LIS3MDLData.h>
#include <sensors/MS580301BA07/MS580301BA07Data.h>
#include <sensors/analog/battery/BatteryVoltageSensorData.h>
#include <sensors/analog/current/CurrentSensorData.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130AData.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAAData.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDAData.h>

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL_sensors/HILSensors.h>
#endif

namespace DeathStackBoard
{

/**
 * @brief This class contains an instance of each mavlink packet that can be
 * sent by the OBSW and related getter/setter functions.
 *
 * WARNING: These packets are updated by the LoggerService. If the
 * LoggerService is not active, the values inside packets WILL NOT BE UPDATED.
 */
class TmRepository : public Singleton<TmRepository>
{
    friend class Singleton<TmRepository>;

public:
    /**
     * @brief Default update function.
     *
     * Later in this file you will find a template-specialized version of this
     * function for each loggable struct that has to be sent via telemetry.
     *
     * @tparam T type of the logged struct
     * @param t reference to the logged struct
     */
    template <typename T>
    inline void update(const T& t)
    {
        UNUSED(t);
        return;
    }

    /**
     * @brief Retrieve a telemetry message in packed form.
     *
     * @param req_tm    required telemetry
     * @param sys_id    system id to pack it with
     * @param comp_id   component id to pack it with
     * @return          packed mavlink struct of that telemetry or a NACK_TM if
     *                  the telemetry id was not found.
     */
    mavlink_message_t packTM(uint8_t req_tm, uint8_t sys_id = 0,
                             uint8_t comp_id = 0);

    void sendTelemetry(MavDriver* mav_driver, const uint8_t tm_id);

private:
    TmRepository() {}
    ~TmRepository() {}

    /**
     * @brief Struct containing all TMs in the form of mavlink messages.
     */
    struct TmRepository_t
    {
        mavlink_sys_tm_t sys_tm;
        mavlink_pin_obs_tm_t pin_obs_tm;
        mavlink_logger_tm_t logger_tm;
        mavlink_fmm_tm_t fmm_tm;
        mavlink_tmtc_tm_t tmtc_tm;
        mavlink_task_stats_tm_t task_stats_tm;
        mavlink_dpl_tm_t dpl_tm;
        mavlink_ada_tm_t ada_tm;
        mavlink_abk_tm_t abk_tm;
        mavlink_nas_tm_t nas_tm;

        mavlink_can_tm_t can_tm;
        mavlink_strain_board_tm_t strain_board_tm;

        mavlink_ms5803_tm_t digital_baro_tm;
        mavlink_bmx160_tm_t bmx_tm;
        mavlink_lis3mdl_tm_t lis3mdl_tm;
        mavlink_adc_tm_t adc_tm;
        mavlink_gps_tm_t gps_tm;

        mavlink_hr_tm_t hr_tm;
        mavlink_lr_tm_t lr_tm;
        mavlink_windtunnel_tm_t wind_tm;
        mavlink_sensors_tm_t sensors_tm;
        mavlink_test_tm_t test_tm;
    } tm_repository;

    uint8_t curHrIndex = 0;

    FlightStatsRecorder stats_rec;

    PrintLogger log = Logging::getLogger("deathstack.tmrepo");
};

template <>
void TmRepository::update<AirBrakesControllerStatus>(
    const AirBrakesControllerStatus& t);

template <>
void TmRepository::update<AirBrakesChosenTrajectory>(
    const AirBrakesChosenTrajectory& t);

template <>
void TmRepository::update<AirBrakesData>(const AirBrakesData& t);

template <>
void TmRepository::update<AirBrakesAlgorithmData>(
    const AirBrakesAlgorithmData& t);

template <>
void TmRepository::update<ADS1118Data>(const ADS1118Data& t);

template <>
void TmRepository::update<MS5803Data>(const MS5803Data& t);

template <>
void TmRepository::update<MPXHZ6130AData>(const MPXHZ6130AData& t);

template <>
void TmRepository::update<SSCDRRN015PDAData>(const SSCDRRN015PDAData& t);

template <>
void TmRepository::update<AirSpeedPitot>(const AirSpeedPitot& t);

template <>
void TmRepository::update<SSCDANN030PAAData>(const SSCDANN030PAAData& t);

#ifndef HARDWARE_IN_THE_LOOP
template <>
void TmRepository::update<BMX160WithCorrectionData>(
    const BMX160WithCorrectionData& t);
#endif

template <>
void TmRepository::update<BMX160Temperature>(const BMX160Temperature& t);

template <>
void TmRepository::update<LIS3MDLData>(const LIS3MDLData& t);

#ifndef HARDWARE_IN_THE_LOOP
template <>
void TmRepository::update<UbloxGPSData>(const UbloxGPSData& t);
#endif

template <>
void TmRepository::update<SensorsStatus>(const SensorsStatus& t);

/**
 * @brief Battery status, sampled by internal ADC.
 */
template <>
void TmRepository::update<BatteryVoltageSensorData>(
    const BatteryVoltageSensorData& t);

/**
 * @brief Motor current sense, sampled by internal ADC.
 */
template <>
void TmRepository::update<CurrentSensorData>(const CurrentSensorData& t);

template <>
void TmRepository::update<Xbee::ATCommandResponseFrameLog>(
    const Xbee::ATCommandResponseFrameLog& t);

/**
 * @brief Logger.
 */
template <>
void TmRepository::update<LogStats>(const LogStats& t);

/**
 * @brief Initialization status of the board.
 */
template <>
void TmRepository::update<DeathStackStatus>(const DeathStackStatus& t);

/**
 * @brief Flight Mode Manager.
 */
template <>
void TmRepository::update<FMMStatus>(const FMMStatus& t);

/**
 * @brief Navigation System.
 */
template <>
void TmRepository::update<NASStatus>(const NASStatus& t);

template <>
void TmRepository::update<NASKalmanState>(const NASKalmanState& t);

template <>
void TmRepository::update<NASReferenceValues>(const NASReferenceValues& t);

template <>
void TmRepository::update<NASTriadResult>(const NASTriadResult& t);

/**
 * @brief Launch and Nosecone detachment pins and DPL servo optical sensor.
 */
template <>
void TmRepository::update<PinStatus>(const PinStatus& t);

/**
 * @brief TMTCController (Mavlink).
 */
template <>
void TmRepository::update<MavlinkStatus>(const MavlinkStatus& t);

/**
 * @brief Sensors.
 */
// template <>
// void TmRepository::update<SensorStatus>(const SensorStatus& t);

/**
 * @brief Deployment Controller.
 */
template <>
void TmRepository::update<DeploymentStatus>(const DeploymentStatus& t);

/**
 * @brief ADA state machine.
 */
template <>
void TmRepository::update<ADAControllerStatus>(const ADAControllerStatus& t);

/**
 * @brief ADA target dpl pressure.
 */
template <>
void TmRepository::update<TargetDeploymentAltitude>(
    const TargetDeploymentAltitude& t);

/**
 * @brief ADA kalman filter values.
 */
template <>
void TmRepository::update<ADAKalmanState>(const ADAKalmanState& t);

// /* ADA kalman altitude values */
template <>
void TmRepository::update<ADAData>(const ADAData& t);

/**
 * @brief ADA calibration reference values set by TC.
 */
template <>
void TmRepository::update<ADAReferenceValues>(const ADAReferenceValues& t);

/**
 * @brief System statistics.
 */
template <>
void TmRepository::update<SystemData>(const SystemData& t);

/**
 * @brief Sensor Manager scheduler.
 */
template <>
void TmRepository::update<TaskStatResult>(const TaskStatResult& t);

/**
 * @brief FlightStatsRecorder liftoff stats.
 */
template <>
void TmRepository::update<LiftOffStats>(const LiftOffStats& t);

/**
 * @brief FlightStatsRecorder apogee stats.
 */
template <>
void TmRepository::update<ApogeeStats>(const ApogeeStats& t);

/**
 * @brief FlightStatsRecorder drogue deployment stats.
 */
template <>
void TmRepository::update<DrogueDPLStats>(const DrogueDPLStats& t);

/**
 * @brief FlightStatsRecorder main deployment stats.
 */
template <>
void TmRepository::update<MainDPLStats>(const MainDPLStats& t);

/**
 * @brief FlightStatsRecorder hbridge test stats.
 */
template <>
void TmRepository::update<CutterTestStats>(const CutterTestStats& t);

#ifdef HARDWARE_IN_THE_LOOP
template <>
void TmRepository::update<HILImuData>(const HILImuData& t);

template <>
void TmRepository::update<HILBaroData>(const HILBaroData& t);

template <>
void TmRepository::update<HILGpsData>(const HILGpsData& t);
#endif

}  // namespace DeathStackBoard