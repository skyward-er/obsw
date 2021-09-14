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

#pragma once

#include <ApogeeDetectionAlgorithm/ADAData.h>
#include <kalman/KalmanEigen.h>
#include <math/Stats.h>
#include <sensors/Sensor.h>

namespace DeathStackBoard
{

class ADAAlgorithm
{
public:
    struct AltitudeAGL
    {
        float altitude;
    };

    ADAAlgorithm(ADAReferenceValues ref_values);

    ~ADAAlgorithm();

    void updateBaro(float pressure);

    void updateGPS(float lat, float lon, bool fix);

    ADAKalmanState getKalmanState();

    ADAData getADAData() const { return ada_data; }

    /**
     * @brief Current altitude above mean sea level.
     *
     * @return Altitude in meters (MSL).
     */
    float getAltitudeMsl() const;

    /**
     * @brief Returns altitude for main chute deployment.
     *
     * Checks if last gps sample has fix, returns altitude above ground level,
     * if not returns QFE altitude relative to the elevation of the launch site.
     *
     * @return Altitude in meters.
     */
    float getAltitudeForDeployment() const;

    /**
     * @brief Current vertical speed in m/s, positive upwards.
     */
    float getVerticalSpeed() const;

    /**
     * @brief Converts an atmospheric pressure to altitude based on the provided
     * reference values.
     *
     * @param pressure Atmospheric pressure in Pa.
     * @return Corresponding altitude above mean sea level (m).
     */
    float pressureToAltitude(float pressure);

    /**
     * @brief Converts an altitude above mean sea level to altitude for chute
     * deployment (see getAltitudeForDeployment()).
     */
    float altitudeMSLtoAGL(float altitude_msl) const;

    ADAReferenceValues getReferenceValues() const { return ref_values; }

private:
    void updatePressureKalman(float pressure);

    /**
     * @brief Method to initialize the kalman configuration structure
     */
    const KalmanEigen<float, KALMAN_STATES_NUM,
                      KALMAN_OUTPUTS_NUM>::KalmanConfig
    getKalmanConfig(const float ref_pressure);

    // References for pressure to altitude conversion
    ADAReferenceValues ref_values;

    KalmanEigen<float, KALMAN_STATES_NUM, KALMAN_OUTPUTS_NUM> filter;

    ADAData ada_data;

    float last_lat = 0;
    float last_lon = 0;
    bool last_fix  = false;

#ifdef DEBUG
    unsigned int counter = 0;
#endif
};

}  // namespace DeathStackBoard