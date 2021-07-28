/**
 * Copyright (c) 2019-2020 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#include <Singleton.h>
#include <drivers/Xbee/APIFramesLog.h>
#include <drivers/adc/ADS1118/ADS1118Data.h>
#include <drivers/gps/ublox/UbloxGPSData.h>
#include <sensors/BMX160/BMX160Data.h>
#include <sensors/LIS3MDL/LIS3MDLData.h>
#include <sensors/MS580301BA07/MS580301BA07Data.h>
#include <sensors/analog/battery/BatteryVoltageSensorData.h>
#include <sensors/analog/current/CurrentSensorData.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130AData.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAAData.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDAData.h>

#include "ADA/ADAData.h"
#include "AeroBrakesController/AeroBrakesData.h"
#include "AeroBrakesController/WindData.h"
#include "DeathStackStatus.h"
#include "DeploymentController/DeploymentData.h"
#include "FlightModeManager/FMMStatus.h"
#include "NavigationSystem/NASData.h"
#include "PinHandler/PinHandlerData.h"
#include "Radio/Mavlink.h"

#ifdef HARDWARE_IN_THE_LOOP
#include "hardware_in_the_loop/HIL_sensors/HILSensors.h"
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
     * Later in this file you will find a template-specialized version of this
     * function for each loggable struct that has to be sent via telemetry.
     * @tparam T type of the logged struct
     * @param t reference to the logged struct
     */
    template <typename T>
    inline void update(const T& t)
    {
        UNUSED(t);
        return;
    }

    // /**
    //  * @brief Add a new packed HR packet to the HR telemetry message.
    //  * The telemetry message contains multiple HR packets to reduce the
    //  impact
    //  * of mavlink overhead.
    //  * @return true if the HR message is full, i.e. ready to be sent
    //  */
    // bool updateHR();

    // /**
    //  * @brief Pack the current LR packet into a mavlink message.
    //  */
    // void updateLR();

    /**
     * Retrieve a telemetry message in packed form.
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

    /* Struct containing all TMs in the form of mavlink messages */
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
        mavlink_adc_tm_t adc_tm;
        mavlink_gps_tm_t gps_tm;

        mavlink_hr_tm_t hr_tm;
        mavlink_lr_tm_t lr_tm;
        mavlink_windtunnel_tm_t wind_tm;
        mavlink_sensors_tm_t sensors_tm;
        mavlink_test_tm_t test_tm;
    } tm_repository;

    uint8_t curHrIndex = 0;

    /* Temporary packet to hold HR_TM values before bitpacking */
    // struct HighRatePacket_t
    // {
    //     long long timestamp;
    //     float pressure_ada;
    //     float pressure_digi;
    //     float msl_altitude;
    //     float agl_altitude;
    //     float vert_speed;
    //     float vert_speed_2;
    //     float acc_x;
    //     float acc_y;
    //     float acc_z;
    //     float gyro_x;
    //     float gyro_y;
    //     float gyro_z;
    //     float gps_lat;
    //     float gps_lon;
    //     float gps_alt;
    //     float temperature;
    //     uint8_t fmm_state;
    //     uint8_t dpl_state;
    //     uint8_t pin_launch;
    //     uint8_t pin_nosecone;
    //     uint8_t gps_fix;
    // } hr_pkt;

    /* Temporary packet to hold LR_TM values before bitpacking */
    // struct LowRatePacket_t
    // {
    //     long long liftoff_ts;
    //     long long liftoff_max_acc_ts;
    //     float liftoff_max_acc;
    //     long long max_zspeed_ts;
    //     float max_zspeed;
    //     float max_speed_altitude;
    //     long long apogee_ts;
    //     float nxp_min_pressure;
    //     float hw_min_pressure;
    //     float kalman_min_pressure;
    //     float digital_min_pressure;
    //     float baro_max_altitutde;
    //     float gps_max_altitude;
    //     float apogee_lat;
    //     float apogee_lon;
    //     long long drogue_dpl_ts;
    //     float drogue_dpl_max_acc;
    //     long long main_dpl_ts;
    //     float main_dpl_altitude;
    //     float main_dpl_zspeed;
    //     float main_dpl_acc;
    // } lr_pkt;
};

/*
 * Each function here is an implementation of the update() method for a
 * specific status struct.
 */

template <>
void TmRepository::update<AeroBrakesData>(const AeroBrakesData& t);

template <>
void TmRepository::update<WindData>(const WindData& t);

// template <>
// void TmRepository::update<ADS1118Data>(const ADS1118Data& t);

template <>
void TmRepository::update<MS5803Data>(const MS5803Data& t);

template <>
void TmRepository::update<MPXHZ6130AData>(const MPXHZ6130AData& t);

template <>
void TmRepository::update<SSCDRRN015PDAData>(const SSCDRRN015PDAData& t);

template <>
void TmRepository::update<SSCDANN030PAAData>(const SSCDANN030PAAData& t);

template <>
void TmRepository::update<BMX160Data>(const BMX160Data& t);

template <>
void TmRepository::update<BMX160Temperature>(const BMX160Temperature& t);

template <>
void TmRepository::update<LIS3MDLData>(const LIS3MDLData& t);

template <>
void TmRepository::update<UbloxGPSData>(const UbloxGPSData& t);

/* Battery status, sampled by internal ADC */
template <>
void TmRepository::update<BatteryVoltageSensorData>(
    const BatteryVoltageSensorData& t);

/* Motor current sense, sampled by internal ADC */
template <>
void TmRepository::update<CurrentSensorData>(const CurrentSensorData& t);

template <>
void TmRepository::update<Xbee::ATCommandResponseFrameLog>(
    const Xbee::ATCommandResponseFrameLog& t);

/* Logger */
template <>
void TmRepository::update<LogStats>(const LogStats& t);

/* Initialization status of the board*/
template <>
void TmRepository::update<DeathStackStatus>(const DeathStackStatus& t);

/* Flight Mode Manager */
template <>
void TmRepository::update<FMMStatus>(const FMMStatus& t);

/* Navigation System */
template <>
void TmRepository::update<NASStatus>(const NASStatus& t);

template <>
void TmRepository::update<NASKalmanState>(const NASKalmanState& t);

/* Launch and Nosecone detachment pins and DPL servo optical sensor */
template <>
void TmRepository::update<PinStatus>(const PinStatus& t);

// /* TMTCManager (Mavlink) */
// template <>
// void TmRepository::update<MavlinkStatus>(const MavlinkStatus& t);

/* Deployment Controller */
template <>
void TmRepository::update<DeploymentStatus>(const DeploymentStatus& t);

/* ADA state machine */
template <>
void TmRepository::update<ADAControllerStatus>(const ADAControllerStatus& t);

// /* ADA target dpl pressure */
// template <>
// void TmRepository::update<TargetDeploymentAltitude>(
//     const TargetDeploymentAltitude& t);

/* ADA kalman filter values */
template <>
void TmRepository::update<ADAKalmanState>(const ADAKalmanState& t);

// /* ADA kalman altitude values */
template <>
void TmRepository::update<ADAData>(const ADAData& t);

// /* ADA calibration reference values set by TC */
// template <>
// void TmRepository::update<ReferenceValues>(const ReferenceValues& t);

// /* Digital Pressure Sensor */
// template <>
// void TmRepository::update<MS5803Data>(const MS5803Data& t);

// /* GPS */
// template <>
// void TmRepository::update<PiksiData>(const PiksiData& t);

// /* Sensor Manager scheduler */
// template <>
// void TmRepository::update<TaskStatResult>(const TaskStatResult& t);

// /* FlightStatsRecorder liftoff stats */
// template <>
// void TmRepository::update<LiftOffStats>(const LiftOffStats& t);

// /* FlightStatsRecorder apogee stats */
// template <>
// void TmRepository::update<ApogeeStats>(const ApogeeStats& t);

// /* FlightStatsRecorder deployment stats */
// template <>
// void TmRepository::update<DrogueDPLStats>(const DrogueDPLStats& t);

// /* FlightStatsRecorder hbridge test stats */
// template <>
// void TmRepository::update<CutterTestStats>(const CutterTestStats& t);

#ifdef HARDWARE_IN_THE_LOOP
template <>
void TmRepository::update<HILImuData>(const HILImuData& t);

template <>
void TmRepository::update<HILBaroData>(const HILBaroData& t);

template <>
void TmRepository::update<HILGpsData>(const HILGpsData& t);
#endif

}  // namespace DeathStackBoard