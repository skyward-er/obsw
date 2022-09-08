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
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>

#include <Eigen/Core>

using namespace Boardcore;
using namespace Eigen;

namespace Payload
{
FlightStatsRecorder::FlightStatsRecorder()
{
    stats.liftoff_max_acc_ts = 0;
    stats.dpl_ts             = 0;
    stats.max_z_speed_ts     = 0;
    stats.apogee_ts          = 0;
    stats.liftoff_max_acc    = -1;
    stats.dpl_max_acc        = -1;
    stats.max_z_speed        = -1;
    stats.max_airspeed_pitot = -1;
    stats.max_speed_altitude = -1;
    stats.apogee_lat         = -1;
    stats.apogee_lon         = -1;
    stats.apogee_alt         = -1;
    stats.min_pressure       = -1;
}

void FlightStatsRecorder::update(AccelerometerData data)
{
    Vector3f accelerations{data.accelerationX, data.accelerationY,
                           data.accelerationZ};
    float squaredNorm = accelerations.squaredNorm();

    if (FlightModeManager::getInstance().getStatus().state ==
            FlightModeManagerState::ASCENDING &&
        (squaredNorm > stats.liftoff_max_acc || stats.liftoff_max_acc == -1))
    {
        stats.liftoff_max_acc    = squaredNorm;
        stats.liftoff_max_acc_ts = data.accelerationTimestamp;
    }
    else if (FlightModeManager::getInstance().getStatus().state ==
                 FlightModeManagerState::DROGUE_DESCENT &&
             (squaredNorm > stats.dpl_max_acc || stats.dpl_max_acc == -1))
    {
        stats.dpl_max_acc = squaredNorm;
        stats.dpl_ts      = data.accelerationTimestamp;
    }
}

void FlightStatsRecorder::update(NASState state)
{
    if (FlightModeManager::getInstance().getStatus().state ==
            FlightModeManagerState::ASCENDING &&
        (-state.vd > stats.max_z_speed || stats.max_z_speed == -1))
    {
        stats.max_z_speed        = -state.vd;
        stats.max_speed_altitude = -state.d;
    }
}

void FlightStatsRecorder::update(PitotData data)
{
    if (FlightModeManager::getInstance().getStatus().state ==
            FlightModeManagerState::ASCENDING &&
        (data.airspeed > stats.max_airspeed_pitot ||
         stats.max_airspeed_pitot == -1))
        stats.max_airspeed_pitot = data.airspeed;
}

void FlightStatsRecorder::update(PressureData data)
{
    if (FlightModeManager::getInstance().getStatus().state ==
            FlightModeManagerState::ASCENDING &&
        (data.pressure < stats.min_pressure || stats.min_pressure == -1))
        stats.min_pressure = data.pressure;
}

void FlightStatsRecorder::setApogee(GPSData data)
{
    stats.apogee_alt = data.height;
    stats.apogee_lat = data.latitude;
    stats.apogee_lon = data.longitude;
}

mavlink_payload_stats_tm_t FlightStatsRecorder::getStats() { return stats; }

}  // namespace Payload