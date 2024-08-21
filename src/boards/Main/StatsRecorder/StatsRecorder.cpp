/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "StatsRecorder.h"

#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <common/ReferenceConfig.h>
#include <utils/AeroUtils/AeroUtils.h>

using namespace Boardcore;
using namespace Main;
using namespace miosix;
using namespace Eigen;
using namespace Common;

void StatsRecorder::reset()
{
    Lock<FastMutex> lock{statsMutex};
    stats = Stats{};
}

StatsRecorder::Stats StatsRecorder::getStats()
{
    Lock<FastMutex> lock{statsMutex};
    return stats;
}

void StatsRecorder::liftoffDetected(uint64_t ts)
{
    Lock<FastMutex> lock{statsMutex};
    stats.liftoffTs = ts;
}

void StatsRecorder::apogeeDetected(uint64_t ts, float lat, float lon, float alt)
{
    Lock<FastMutex> lock{statsMutex};
    stats.apogeeTs  = ts;
    stats.apogeeLat = lat;
    stats.apogeeLon = lon;
    stats.apogeeAlt = alt;
}

void StatsRecorder::deploymentDetected(uint64_t ts, float alt)
{
    Lock<FastMutex> lock{statsMutex};
    stats.dplTs  = ts;
    stats.dplAlt = alt;
}

void StatsRecorder::updateAcc(const AccelerometerData &data)
{
    auto state = getModule<FlightModeManager>()->getState();

    float length = static_cast<Vector3f>(data).norm();
    Lock<FastMutex> lock{statsMutex};

    if (state == FlightModeManagerState::POWERED_ASCENT ||
        state == FlightModeManagerState::ARMED)
    {
        // Record this event only during liftoff
        if (length > stats.liftoffMaxAcc)
        {
            stats.liftoffMaxAcc   = length;
            stats.liftoffMaxAccTs = data.accelerationTimestamp;
        }
    }
    else if (state == FlightModeManagerState::DROGUE_DESCENT)
    {
        // Record this event only during drogue deployment
        if (length > stats.apogeeMaxAcc)
        {
            stats.apogeeMaxAcc   = length;
            stats.apogeeMaxAccTs = data.accelerationTimestamp;
        }
    }
    else if (state == FlightModeManagerState::TERMINAL_DESCENT)
    {
        // Record this event only during main deployment
        if (length > stats.dplMaxAcc)
        {
            stats.dplMaxAcc   = length;
            stats.dplMaxAccTs = data.accelerationTimestamp;
        }
    }
}

void StatsRecorder::updateNas(const NASState &data)
{
    auto state = getModule<FlightModeManager>()->getState();

    Lock<FastMutex> lock{statsMutex};
    if (state == FlightModeManagerState::POWERED_ASCENT ||
        state == FlightModeManagerState::UNPOWERED_ASCENT ||
        state == FlightModeManagerState::DROGUE_DESCENT ||
        state == FlightModeManagerState::TERMINAL_DESCENT)
    {
        // Record this event only during flight
        float speed = Vector3f{data.vn, data.vd, data.ve}.norm();
        float alt   = -data.d;

        if (speed > stats.maxSpeed)
        {
            stats.maxSpeed    = speed;
            stats.maxSpeedAlt = alt;
            stats.maxSpeedTs  = data.timestamp;
        }

        // TODO: Grab ref temperature from global ReferenceValues
        float mach = Aeroutils::computeMach(
            data.d, speed,
            ReferenceConfig::defaultReferenceValues.refTemperature);

        if (mach > stats.maxMach)
        {
            stats.maxMach   = mach;
            stats.maxMachTs = data.timestamp;
        }
    }
}

void StatsRecorder::updateDplPressure(const PressureData &data)
{
    auto state = getModule<FlightModeManager>()->getState();

    Lock<FastMutex> lock{statsMutex};
    if (state == FlightModeManagerState::DROGUE_DESCENT ||
        state == FlightModeManagerState::TERMINAL_DESCENT)
    {
        // Record this event only in drogue or terminal descent
        if (data.pressure > stats.maxDplPressure)
        {
            stats.maxDplPressure   = data.pressure;
            stats.maxDplPressureTs = data.pressureTimestamp;
        }
    }
}