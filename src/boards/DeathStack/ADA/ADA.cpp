/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Conterio
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
#include "ADA.h"

#include <Debug.h>
#include <configs/ADAconfig.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <utils/aero/AeroUtils.h>

#include "DeploymentUtils/elevation_map.h"
#include "TimestampTimer.h"
#include "diagnostic/CpuMeter.h"

namespace DeathStackBoard
{

using namespace ADAConfigs;

ADA::ADA(ReferenceValues ref_values)
    : ref_values(ref_values),
      filter(ADAConfigs::getKalmanConfig(ref_values.ref_pressure))
{
    TRACE("[ADA] Finalized calibration. p_ref: %.3f, p0: %.3f, t0: %.3f\n",
          ref_values.ref_pressure, ref_values.msl_pressure,
          ref_values.msl_temperature);
}

ADA::~ADA() {}

void ADA::updateBaro(float pressure)
{
    updatePressureKalman(pressure);

    // Convert filter data to altitudes & speeds
    ada_data.timestamp    = TimestampTimer::getTimestamp();
    ada_data.msl_altitude = pressureToAltitude(filter.getState()(0, 0));

    AltitudeDPL ad               = altitudeMSLtoDPL(ada_data.msl_altitude);
    ada_data.dpl_altitude        = ad.altitude;
    ada_data.is_dpl_altitude_agl = ad.is_agl;

    ada_data.vert_speed = aeroutils::verticalSpeed(
        filter.getState()(0, 0), filter.getState()(1, 0),
        ref_values.msl_pressure, ref_values.msl_temperature);
}

void ADA::updateGPS(float lat, float lon, bool fix)
{
    last_lat = lat;
    last_lon = lon;
    last_fix = fix;
}

float ADA::getAltitudeMsl() const { return ada_data.msl_altitude; }

ADA::AltitudeDPL ADA::getAltitudeForDeployment() const
{
    return AltitudeDPL{ada_data.dpl_altitude, ada_data.is_dpl_altitude_agl};
}

float ADA::getVerticalSpeed() const { return ada_data.vert_speed; }

float ADA::pressureToAltitude(float pressure)
{
    return aeroutils::relAltitude(pressure, ref_values.msl_pressure,
                                  ref_values.msl_temperature);
}

ADA::AltitudeDPL ADA::altitudeMSLtoDPL(float altitude_msl) const
{
    float elev = elevationmap::getElevation(last_lat, last_lon);

    if (last_fix && elev >= 0)
    {
        return {altitude_msl - elev, true};
    }
    else
    {
        return {altitude_msl - ref_values.ref_altitude, false};
    }
}

ADAKalmanState ADA::getKalmanState()
{
    ADAKalmanState state;
    state.timestamp = TimestampTimer::getTimestamp();

    state.x0 = filter.getState()(0, 0);
    state.x1 = filter.getState()(1, 0);
    state.x2 = filter.getState()(2, 0);

    return state;
}

void ADA::updatePressureKalman(float pressure)
{
    filter.predict();

    CVectorP y(pressure);  // column vector
    if (!filter.correct(y))
    {
        TRACE("[ADA] Kalman correction step failed \n");
    }
}

}  // namespace DeathStackBoard