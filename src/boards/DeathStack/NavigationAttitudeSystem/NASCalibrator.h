/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <Common.h>
#include <Debug.h>
#include <NavigationAttitudeSystem/NASData.h>
#include <math/Stats.h>
#include <miosix.h>

namespace DeathStackBoard
{

class NASCalibrator
{
public:
    NASCalibrator(const uint32_t n_samples);

    bool calibIsComplete();

    /**
     * @brief Adds a pressure sample to the stats
     */
    void addBaroSample(float p);

    /**
     * @brief Adds a GPS sample to the stats
     */
    void addGPSSample(float lat, float lon);

    /**
     * @brief Adds an acceleration sample to the stats
     */
    void addAccelSample(float x, float y, float z);

    /**
     * @brief Adds an acceleration sample to the stats
     */
    void addMagSample(float x, float y, float z);

    /**
     * @brief Set temperature of the launch location
     */
    void setReferenceTemperature(float t);


    /**
     * @brief Set latitude/longitude of the launchpad
     */
    void setReferenceCoordinates(float lat, float lon);

    /**
     * @brief Reset stats
     */
    void reset();

    NASReferenceValues getReferenceValues();

private:
    const uint32_t n_samples;

    NASReferenceValues ref_values{};

    Stats pressure_stats;  // Computes mean std dev etc for calibration

    Stats gps_lat_stats;
    Stats gps_lon_stats;

    Stats accel_x_stats;
    Stats accel_y_stats;
    Stats accel_z_stats;

    Stats mag_x_stats;
    Stats mag_y_stats;
    Stats mag_z_stats;

    // Refernece flags
    bool ref_coordinates_set  = false;
    bool ref_temperature_set = false;
};

}  // namespace DeathStackBoard