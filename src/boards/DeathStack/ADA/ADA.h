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

#pragma once

#include <kalman/Kalman.h>
#include <math/Stats.h>
#include "ADAStatus.h"

namespace DeathStackBoard
{
class ADA
{
public:
    struct AltitudeDPL
    {
        float altitude;
        bool is_agl;
    };

    ADA(ReferenceValues setup_data);
    ~ADA();

    void updateBaro(float pressure);
    void updateAcc(float ax);
    void updateGPS(double lat, double lon, bool has_fix);

    inline KalmanState getKalmanState() const;

    ADAData getADAData() const
    {
        return ada_data;
    }

    /**
     * @brief Current altitude above mean sea level
     *
     * @return Altitude in meters (MSL)
     */
    float getAltitudeMsl() const;

    /**
     * @brief Returns altitude for main chute deployment altitude check:
     * if last gps sample has fix, returns altitude above ground level
     * if not, returns QFE altitude relative to the elevation of the launch
     * site
     *
     * @return Altitude in meters
     */
    AltitudeDPL getAltitudeForDeployment() const;

    /**
     * @brief Current vertical speed in m/s, positive upwards
     */
    float getVerticalSpeed() const;

    /**
     * @brief Converts an atmospheric pressure to altitude based on the provided
     * reference values.
     *
     * @param    pressure Atmospheric pressure in Pa
     * @return Corresponding altitude above mean sea level (m)
     */
    float pressureToAltitude(float pressure) const;

    /**
     * @brief Converts an altitude above mean sea level to altitude for chute
     * deployment (see getAltitudeForDeployment())
     *
     */
    AltitudeDPL altitudeMSLtoDPL(float altitude_msl) const;

    ReferenceValues getReferenceValues() const { return ref_values; }

private:
    Kalman<3, 1> filter;      // Filter object
    Kalman<3, 2> filter_acc;  // Filter with accelerometer
    // Stats for acceleration averaging
    Stats acc_stats;

    // References for pressure to altitude conversion
    ReferenceValues ref_values;

    ADAData ada_data;

    float last_acc_average = 0;
    
    double last_lat = 0;
    double last_lon = 0;
    bool last_fix = false;
};
}  // namespace DeathStackBoard