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

namespace DeathStackBoard
{
ADA::ADA(ADASetupData setup_data)
    : filter(A_INIT, C_INIT, V1_INIT, V2_INIT, P_INIT),
      filter_acc(A_INIT, C_INIT_ACC, V1_INIT_ACC, V2_INIT_ACC, P_INIT_ACC)
{
    float pressure_ref = setup_data.pressure_stats_results.mean;

    // Initialize Kalman filter
    filter.X(0, 0) = pressure_ref;
    filter.X(1, 0) = 0;
    filter.X(2, 0) = KALMAN_INITIAL_ACCELERATION;

    filter_acc.X(0, 0) = ref_values.ref_altitude;
    filter_acc.X(1, 0) = 0;
    filter_acc.X(2, 0) = 0;

    // Calculat MSL values for altitude calculation
    ref_values.msl_pressure = aeroutils::mslPressure(
        pressure_ref, ref_values.ref_temperature, ref_values.ref_altitude);

    ref_values.msl_temperature = aeroutils::mslTemperature(
        ref_values.ref_temperature, ref_values.ref_altitude);

    TRACE("[ADA] Finalized calibration. p_ref: %.3f, p0: %.3f, t0: %.3f\n",
          pressure_ref, ref_values.msl_pressure, ref_values.msl_temperature);

    // ADA READY!
    sEventBroker->post({EV_ADA_READY}, TOPIC_ADA);
}

ADA::~ADA() {}

void ADA::updateBaro(float pressure)
{
    MatrixBase<float, 1, 1> y{pressure};
    filter.update(y);

    float z  = aeroutils::relAltitude(pressure, ref_values.msl_pressure,
                                     ref_values.msl_temperature);
    float ax = (acc_stats.getStats().mean - 1) *
               9.81;  // Remove graviti vector and convert gs to m/s^2
    MatrixBase<float, 2, 1> y_acc{z, ax};
    filter_acc.update(y_acc);

    acc_stats.reset();
    // TRACE("[ADA] Updated filter with %f\n", pressure);
}

void ADA::updateAcc(float ax) { acc_stats.add(ax); }

float ADA::getAltitude()
{
    return aeroutils::relAltitude(filter.X(0, 0), ref_values.msl_pressure,
                                  ref_values.msl_temperature);
}

float ADA::getVerticalSpeed()
{
    return aeroutils::verticalSpeed(filter.X(0, 0), filter.X(1, 0),
                                    ref_values.msl_pressure,
                                    ref_values.msl_temperature);
}

KalmanState ADA::getKalmanState()
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