/* Copyright (c) 2019-2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Federico Mandelli
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/FlightStatsRecorderConfig.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>

#include <Eigen/Core>

using namespace Boardcore;
using namespace Payload::FlightStatsRecorderConfig;
using namespace Eigen;
using namespace miosix;

namespace Payload
{
FlightStatsRecorder::FlightStatsRecorder()
{
    // Init the data structure to avoid UB
    stats.max_airspeed_pitot = 0;
    stats.apogee_alt         = 0;
    stats.apogee_lat         = 0;
    stats.apogee_lon         = 0;
    stats.apogee_ts          = 0;
    stats.cpu_load           = 0;
    stats.dpl_max_acc        = 0;
    stats.dpl_ts             = 0;
    stats.free_heap          = 0;
    stats.liftoff_max_acc    = 0;
    stats.liftoff_max_acc_ts = 0;
    stats.max_z_speed        = 0;
    stats.max_z_speed_ts     = 0;
    stats.min_pressure       = 0;
    stats.max_speed_altitude = 0;
}

bool FlightStatsRecorder::start()
{
    auto& scheduler = getModule<BoardScheduler>()->flightStatsRecorder();

    return scheduler.addTask([this] { update(); }, FSR_UPDATE_PERIOD,
                             TaskScheduler::Policy::RECOVER);
}

void FlightStatsRecorder::update()
{
    // FMM state
    FlightModeManagerState flightState =
        getModule<FlightModeManager>()->getStatus().state;

    // Data gathering
    Sensors* sensor           = getModule<Sensors>();
    AccelerometerData accData = sensor->getIMULastSample().accData;
    PressureData baroData     = sensor->getStaticPressure();
    PitotData pitotData       = sensor->getPitotLastSample();
    GPSData gpsData           = sensor->getUBXGPSLastSample();
    NASState nasData          = getModule<NASController>()->getNasState();

    // Store the apogee ts
    uint64_t previousApogee = stats.apogee_ts;

    // Update the data
    updateAcc(flightState, accData);
    updateBaro(flightState, baroData);
    updatePitot(flightState, pitotData);
    updateNAS(flightState, nasData);

    // If the apogee ts is different update the GPS apogee coordinates
    if (previousApogee != stats.apogee_ts)
    {
        Lock<FastMutex> l(mutex);
        // minus to get positive altitude
        stats.apogee_alt = -nasData.d;
        stats.apogee_lat = gpsData.latitude;
        stats.apogee_lon = gpsData.longitude;
    }
}

mavlink_payload_stats_tm_t FlightStatsRecorder::getStats()
{
    Lock<FastMutex> l(mutex);
    return stats;
}

void FlightStatsRecorder::updateAcc(FlightModeManagerState flightState,
                                    AccelerometerData data)
{
    Vector3f accelerations = static_cast<Vector3f>(data);
    float norm             = accelerations.norm();

    if (flightState == FlightModeManagerState::ASCENDING)
    {
        Lock<FastMutex> l(mutex);

        stats.liftoff_max_acc = std::max(stats.liftoff_max_acc, norm);
        bool changed          = stats.liftoff_max_acc == norm;

        // In case of a change update the timestamp
        stats.liftoff_max_acc_ts =
            changed ? data.accelerationTimestamp : stats.liftoff_max_acc_ts;
    }
    else if (flightState >= FlightModeManagerState::DROGUE_DESCENT)
    {
        Lock<FastMutex> l(mutex);

        stats.dpl_max_acc = std::max(stats.dpl_max_acc, norm);

        // Change the timestamp of DPL if not already done
        stats.dpl_ts =
            stats.dpl_ts == 0 ? data.accelerationTimestamp : stats.dpl_ts;
    }
}

void FlightStatsRecorder::updateBaro(FlightModeManagerState flightState,
                                     PressureData data)
{
    Lock<FastMutex> l(mutex);

    if (stats.min_pressure == 0)
    {
        stats.min_pressure = data.pressure;
    }

    // Update the min
    stats.min_pressure = std::min(stats.min_pressure, data.pressure);
}

void FlightStatsRecorder::updatePitot(FlightModeManagerState flightState,
                                      PitotData data)
{
    Lock<FastMutex> l(mutex);

    stats.max_airspeed_pitot =
        std::max(stats.max_airspeed_pitot, data.airspeed);
}

void FlightStatsRecorder::updateNAS(FlightModeManagerState flightState,
                                    NASState state)
{
    if (flightState == FlightModeManagerState::ASCENDING)
    {
        Lock<FastMutex> l(mutex);

        stats.max_z_speed = std::max(stats.max_z_speed, -state.vd);
        bool changed      = stats.max_z_speed == -state.vd;

        // Change the max speed altitude and timestamp if the max speed changed
        stats.max_speed_altitude =
            changed ? -state.d : stats.max_speed_altitude;

        stats.max_z_speed_ts = changed ? state.timestamp : stats.max_z_speed_ts;
    }
}

}  // namespace Payload
