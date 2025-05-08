/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include "FlightStatsRecorder.h"

#include <Parafoil/Configs/NASConfig.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <common/ReferenceConfig.h>
#include <utils/AeroUtils/AeroUtils.h>

using namespace Boardcore;
using namespace Boardcore::Units::Speed;
using namespace Boardcore::Units::Length;
using namespace Boardcore::Units::Pressure;
using namespace Boardcore::Units::Acceleration;
using namespace miosix;
using namespace Eigen;
using namespace Common;

namespace Parafoil
{

void FlightStatsRecorder::reset()
{
    Lock<FastMutex> lock{statsMutex};
    stats = Stats{};
}

FlightStatsRecorder::Stats FlightStatsRecorder::getStats()
{
    Lock<FastMutex> lock{statsMutex};
    return stats;
}

void FlightStatsRecorder::dropDetected(uint64_t ts)
{
    Lock<FastMutex> lock{statsMutex};
    stats.dropTs = ts;
}

void FlightStatsRecorder::deploymentDetected(uint64_t ts, Meter alt)
{
    Lock<FastMutex> lock{statsMutex};
    stats.dplTs  = ts;
    stats.dplAlt = alt;
}

void FlightStatsRecorder::updateAcc(const AccelerometerData& data)
{
    auto fmmState = getModule<FlightModeManager>()->getState();
    auto wcState  = getModule<WingController>()->getState();

    // Do nothing if it was not dropped yet
    if (fmmState != FlightModeManagerState::FLYING_WING_DESCENT)
        return;

    auto acc = MeterPerSecondSquared{static_cast<Vector3f>(data).norm()};
    Lock<FastMutex> lock{statsMutex};

    if (wcState == WingControllerState::IDLE)
    {
        // Record this event only after drop, before deployment
        if (acc > stats.dropMaxAcc)
        {
            stats.dropMaxAcc   = acc;
            stats.dropMaxAccTs = data.accelerationTimestamp;
        }
    }
    else if (wcState == WingControllerState::FLYING_DEPLOYMENT ||
             wcState == WingControllerState::FLYING_CONTROLLED_DESCENT)
    {
        // Record this event only after deployment
        if (acc > stats.dplMaxAcc)
        {
            stats.dplMaxAcc   = acc;
            stats.dplMaxAccTs = data.accelerationTimestamp;
        }
    }
}

void FlightStatsRecorder::updateNas(const NASState& data, float refTemperature)
{
    auto fmmState = getModule<FlightModeManager>()->getState();
    auto wcState  = getModule<WingController>()->getState();

    // Do nothing if it was not dropped yet
    if (fmmState != FlightModeManagerState::FLYING_WING_DESCENT)
        return;

    Lock<FastMutex> lock{statsMutex};

    if (wcState == WingControllerState::IDLE ||
        wcState == WingControllerState::FLYING_DEPLOYMENT ||
        wcState == WingControllerState::FLYING_CONTROLLED_DESCENT)
    {
        // Record this event only during flight
        auto speed = MeterPerSecond{Vector3f{data.vn, data.vd, data.ve}.norm()};
        auto alt   = Meter{-data.d};

        if (speed > stats.maxSpeed)
        {
            stats.maxSpeed    = speed;
            stats.maxSpeedAlt = alt;
            stats.maxSpeedTs  = data.timestamp;
        }
    }
}

void FlightStatsRecorder::updatePressure(const PressureData& data)
{
    auto fmmState = getModule<FlightModeManager>()->getState();
    auto wcState  = getModule<WingController>()->getState();

    // Do nothing if it was not dropped yet
    if (fmmState != FlightModeManagerState::FLYING_WING_DESCENT)
        return;

    Lock<FastMutex> lock{statsMutex};

    if (wcState == WingControllerState::IDLE ||
        wcState == WingControllerState::FLYING_DEPLOYMENT ||
        wcState == WingControllerState::FLYING_CONTROLLED_DESCENT)
    {
        // Record this event only during flight
        if (stats.minPressure.value() == 0)
            stats.minPressure = Pascal{data.pressure};
        else if (Pascal{data.pressure} < stats.minPressure)
            stats.minPressure = Pascal{data.pressure};
    }
}

}  // namespace Parafoil
