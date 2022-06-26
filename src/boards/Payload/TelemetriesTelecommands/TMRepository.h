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
#pragma once

#include <Payload/TelemetriesTelecommands/Mavlink.h>
#include <Singleton.h>
#include <diagnostic/PrintLogger.h>

namespace Payload
{
class TMRepository : public Boardcore::Singleton<TMRepository>
{
    friend class Boardcore::Singleton<TMRepository>;

public:
    /**
     * @brief Retrieve a system telemetry message in packed form
     *
     * @param reqTm required telemetry
     * @param sysId system if to pack it with
     * @param compId component if to pack it with
     * @return mavlink_message_t packed mavlink struct of that telemetry or a
     * NACK_TM if the telemetry was not found
     */
    mavlink_message_t packSystemTM(uint8_t reqTm,
                                   uint8_t sysId  = TMTC_MAV_SYSID,
                                   uint8_t compId = TMTC_MAV_COMPID);

    /**
     * @brief Retrieve a sensor telemetry message in packed form
     *
     * @param reqTm required telmetry
     * @param sysId system if to pack with it
     * @param compId component if to pack with it
     * @return mavlink_message_t mavlink_message_t packed mavlink struct of that
     * telemetry or a NACK_TM if the telemetry was not found
     */
    mavlink_message_t packSensorTM(uint8_t reqTm,
                                   uint8_t sysId  = TMTC_MAV_SYSID,
                                   uint8_t compId = TMTC_MAV_COMPID);

private:
    /**
     * @brief Struct containing all the TMs in the form of mavlink messages.
     */
    struct TmRepository_t
    {
        // System telemetries
        mavlink_sys_tm_t sysTm;
        mavlink_fsm_tm_t fsmTm;
        mavlink_pin_obs_tm_t pinObsTm;
        mavlink_logger_tm_t loggerTm;
        mavlink_mavlink_stats_tm_t mavlinkStatsTm;
        mavlink_task_stats_tm_t taskStatsTm;
        mavlink_dpl_tm_t dplTm;
        mavlink_ada_tm_t adaTm;
        mavlink_nas_tm_t nasTm;
        mavlink_can_tm_t canTm;
        mavlink_payload_flight_tm_t flightTm;
        mavlink_payload_stats_tm_t stastTm;
        mavlink_sensor_state_tm_t sensorsStateTm;

        // Sensors telemetries
        mavlink_gps_tm_t gpsTm;
        mavlink_imu_tm_t imuTm;
        mavlink_adc_tm_t adcTm;
        mavlink_baro_tm_t barometerTm;
        mavlink_temp_tm_t temperatureTm;
        mavlink_attitude_tm_t attitudeTm;
    } tmRepository;

    /**
     * @brief Logger
     */
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("TMRepository");
};
}  // namespace Payload