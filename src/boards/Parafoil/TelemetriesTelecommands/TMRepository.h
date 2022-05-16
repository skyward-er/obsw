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

#include <Parafoil/TelemetriesTelecommands/Mavlink.h>
#include <Singleton.h>
#include <sensors/BME280/BME280Data.h>
#include <sensors/MPU9250/MPU9250Data.h>
#include <sensors/UbloxGPS/UbloxGPSData.h>

/**
 * @brief This class represents the collection of data that can
 * be sent via radio communication. This refers to mavlink libraries
 * and structures created in the correct .xml file.
 *
 * It is necessary that this singleton class handles the structure update
 * (when a message pack is requested).
 * The pack method is the core of the class. It returns a mavlink_message
 * with the message data(specified with the id) requested.
 */

namespace Parafoil
{

class TMRepository : public Boardcore::Singleton<TMRepository>
{
    friend class Boardcore::Singleton<TMRepository>;

public:
    /**
     * @brief Retrieve a telemetry message in packed form.
     *
     * @param req_tm    required telemetry
     * @param sys_id    system id to pack it with
     * @param comp_id   component id to pack it with
     * @return          packed mavlink struct of that telemetry or a NACK_TM if
     *                  the telemetry id was not found.
     */
    mavlink_message_t packTM(uint8_t req_tm, uint8_t sys_id = TMTC_MAV_SYSID,
                             uint8_t comp_id = TMTC_MAV_COMPID);

    /**
     * @brief Update functions
     */
    void update(Boardcore::MPU9250Data data);
    void update(Boardcore::UbloxGPSData data);
    void update(Boardcore::BME280Data data);

private:
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

    /**
     * @brief Logger
     */
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("TMRepository");
};
}  // namespace Parafoil
