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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <ApogeeDetectionAlgorithm/ADAAlgorithm.h>
#include <configs/ADAConfig.h>
#include <diagnostic/CpuMeter.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <utils/aero/AeroUtils.h>

using namespace Boardcore;

namespace DeathStackBoard
{

using namespace ADAConfigs;

ADAAlgorithm::ADAAlgorithm(ADAReferenceValues ref_values)
    : ref_values(ref_values), filter(getKalmanConfig(ref_values.ref_pressure))
{
    TRACE(
        "[ADA] Initial reference values : alt_ref: %.3f, p_ref: %.3f, t_ref: "
        "%.3f, p0: %.3f, t0: %.3f\n",
        ref_values.ref_altitude, ref_values.ref_pressure,
        ref_values.ref_temperature, ref_values.msl_pressure,
        ref_values.msl_temperature);
}

ADAAlgorithm::~ADAAlgorithm() {}

void ADAAlgorithm::updateBaro(float pressure)
{
    updatePressureKalman(pressure);

    // Convert filter data to altitudes & speeds
    ada_data.timestamp    = TimestampTimer::getInstance().getTimestamp();
    ada_data.msl_altitude = pressureToAltitude(filter.getState()(0, 0));
    ada_data.agl_altitude = altitudeMSLtoAGL(ada_data.msl_altitude);
    ada_data.vert_speed   = aeroutils::verticalSpeed(
          filter.getState()(0, 0), filter.getState()(1, 0),
          ref_values.msl_pressure, ref_values.msl_temperature);

#ifdef DEBUG
    if (counter == 50)
    {
        TRACE("[ADA] z : %.2f - vz : %.2f \n", ada_data.msl_altitude,
              ada_data.vert_speed);
        counter = 0;
    }
    else
    {
        counter++;
    }
#endif
}

void ADAAlgorithm::updateGPS(float lat, float lon, bool fix)
{
    last_lat = lat;
    last_lon = lon;
    last_fix = fix;
}

float ADAAlgorithm::getAltitudeMsl() const { return ada_data.msl_altitude; }

float ADAAlgorithm::getAltitudeForDeployment() const
{
    return ada_data.agl_altitude;
}

float ADAAlgorithm::getVerticalSpeed() const { return ada_data.vert_speed; }

float ADAAlgorithm::pressureToAltitude(float pressure)
{
    return aeroutils::relAltitude(pressure, ref_values.msl_pressure,
                                  ref_values.msl_temperature);
}

float ADAAlgorithm::altitudeMSLtoAGL(float altitude_msl) const
{
    return altitude_msl - ref_values.ref_altitude;
}

ADAKalmanState ADAAlgorithm::getKalmanState()
{
    ADAKalmanState state;
    state.timestamp = TimestampTimer::getInstance().getTimestamp();

    state.x0 = filter.getState()(0, 0);
    state.x1 = filter.getState()(1, 0);
    state.x2 = filter.getState()(2, 0);

    return state;
}

void ADAAlgorithm::updatePressureKalman(float pressure)
{
    filter.predict();
    CVectorP y(pressure);  // column vector
    if (!filter.correct(y))
    {
        TRACE("[ADA] Kalman correction step failed \n");
    }
}

const KalmanEigen<float, KALMAN_STATES_NUM, KALMAN_OUTPUTS_NUM>::KalmanConfig
ADAAlgorithm::getKalmanConfig(const float init_pressure)
{
    KalmanEigen<float, KALMAN_STATES_NUM, KALMAN_OUTPUTS_NUM>::KalmanConfig
        config;
    config.F = F_INIT;
    config.H = H_INIT;
    config.Q = Q_INIT;
    config.R = R_INIT;
    config.P = P_INIT;
    config.x = CVectorN(init_pressure, 0, KALMAN_INITIAL_ACCELERATION);

    return config;
}

}  // namespace DeathStackBoard
