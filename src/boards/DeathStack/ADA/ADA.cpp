/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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
#include <DeathStack/events/Events.h>
#include <Debug.h>
#include <boards/DeathStack/configs/ADA_config.h>
#include <events/EventBroker.h>
#include <libs/simple-template-matrix/matrix.h>
#include <utils/aero/AeroUtils.h>
#include "DeploymentUtils/elevation_map.h"
#include <iostream>

namespace DeathStackBoard
{
ADA::ADA(ReferenceValues ref_values)
    : filter(A_INIT, C_INIT, V1_INIT, V2_INIT, P_INIT),
      filter_acc(A_INIT, C_INIT_ACC, V1_INIT_ACC, V2_INIT_ACC, P_INIT_ACC),
      ref_values(ref_values)
{
    // Initialize Kalman filter
    filter.X(0, 0) = ref_values.ref_pressure;
    filter.X(1, 0) = 0;
    filter.X(2, 0) = KALMAN_INITIAL_ACCELERATION;

    filter_acc.X(0, 0) = ref_values.ref_altitude;
    filter_acc.X(1, 0) = 0;
    filter_acc.X(2, 0) = 0;

    TRACE("[ADA] Finalized calibration. p_ref: %.3f, p0: %.3f, t0: %.3f\n",
          ref_values.ref_pressure, ref_values.msl_pressure,
          ref_values.msl_temperature);
}

ADA::~ADA() {}

void ADA::updateBaro(float pressure)
{
    // First kalman (pressure only)
    MatrixBase<float, 1, 1> y{pressure};
    filter.update(y);

    // Second kalman (pressure and acceleration)
    float z  = pressureToAltitude(pressure);
    float ax = last_acc_average;
    if (ax != NAN)
    {
        ax = 0;
    }
    // if (acc_stats.getStats().nSamples > 0)
    // {
    //     ax = (acc_stats.getStats().mean - 1) *
    //            9.81;  // Remove gravity vector and convert gs to m/s^2
    //     acc_stats.reset();
    // }

    // std::cout << ax << "\n";

    MatrixBase<float, 2, 1> y_acc{z, ax};
    filter_acc.update(y_acc);

    // Convert filter data to altitudes & speeds
    ada_data.timestamp    = miosix::getTick();
    ada_data.msl_altitude = pressureToAltitude(filter.X(0, 0));

    AltitudeDPL ad               = altitudeMSLtoDPL(ada_data.msl_altitude);
    ada_data.dpl_altitude        = ad.altitude;
    ada_data.is_dpl_altitude_agl = ad.is_agl;

    ada_data.vert_speed = aeroutils::verticalSpeed(
        filter.X(0, 0), filter.X(1, 0), ref_values.msl_pressure,
        ref_values.msl_temperature);

    // Filter with accelerometer
    ada_data.acc_msl_altitude = pressureToAltitude(filter_acc.X(0, 0));
    ada_data.acc_vert_speed   = aeroutils::verticalSpeed(
        filter_acc.X(0, 0), filter_acc.X(1, 0), ref_values.msl_pressure,
        ref_values.msl_temperature);
}

void ADA::updateAcc(float ax)
{
    acc_stats.add(ax);
    if (acc_stats.getStats().nSamples >= ACCELERATION_AVERAGE_N_SAMPLES)
    {
        last_acc_average = acc_stats.getStats().mean;
        acc_stats.reset();
    }
}

void ADA::updateGPS(double lat, double lon, bool has_fix)
{
    last_lat = lat;
    last_lon = lon;
    last_fix = has_fix;
}

float ADA::getAltitudeMsl() const { return ada_data.msl_altitude; }

ADA::AltitudeDPL ADA::getAltitudeForDeployment() const
{
    return AltitudeDPL{ada_data.dpl_altitude, ada_data.is_dpl_altitude_agl};
}

float ADA::getVerticalSpeed() const { return ada_data.vert_speed; }

float ADA::pressureToAltitude(float pressure) const
{
    return aeroutils::relAltitude(pressure, ref_values.msl_pressure,
                                  ref_values.msl_temperature);
}

ADA::AltitudeDPL ADA::altitudeMSLtoDPL(float altitude_msl) const
{
    float elev = elevationmap::getElevation(last_lat, last_lon);
    if (last_fix)
    {
        return {altitude_msl - elev, true};
    }
    else
    {
        return {altitude_msl - ref_values.ref_altitude, false};
    }
}

KalmanState ADA::getKalmanState() const
{
    KalmanState state;

    state.x0 = filter.X(0, 0);
    state.x1 = filter.X(1, 0);
    state.x2 = filter.X(2, 0);

    state.x0_acc = filter_acc.X(0, 0);
    state.x1_acc = filter_acc.X(1, 0);
    state.x2_acc = filter_acc.X(2, 0);

    return state;
}
}  // namespace DeathStackBoard