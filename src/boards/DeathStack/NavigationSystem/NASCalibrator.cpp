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

#include "NASCalibrator.h"

//#include <utils/aero/AeroUtils.h>

namespace DeathStackBoard
{

NASCalibrator::NASCalibrator(const uint32_t n_samples = 1000)
    : n_samples(n_samples)
{
}

void NASCalibrator::addBaroSample(float p) { pressure_stats.add(p); }

void NASCalibrator::addGPSSample(float lat, float lon)
{
    gps_lat_stats.add(lat);
    gps_lon_stats.add(lon);
}

void NASCalibrator::addAccelSample(float x, float y, float z)
{
    accel_x_stats.add(x);
    accel_y_stats.add(y);
    accel_z_stats.add(z);
}

void NASCalibrator::addMagSample(float x, float y, float z)
{
    mag_x_stats.add(x);
    mag_y_stats.add(y);
    mag_z_stats.add(z);
}

bool NASCalibrator::calibIsComplete()
{
    // Calibration is complete if enough samples were collected

    return pressure_stats.getStats().nSamples >= n_samples &&
           gps_lat_stats.getStats().nSamples >= n_samples &&
           accel_x_stats.getStats().nSamples >= n_samples &&
           mag_x_stats.getStats().nSamples >= n_samples;
}

void NASCalibrator::reset()
{
    pressure_stats.reset();
    gps_lat_stats.reset();
    gps_lon_stats.reset();
    accel_x_stats.reset();
    accel_y_stats.reset();
    accel_z_stats.reset();
    mag_x_stats.reset();
    mag_y_stats.reset();
    mag_z_stats.reset();
}

NASReferenceValues NASCalibrator::getReferenceValues()
{
    // If calibration is not complete use default values
    if (calibIsComplete())
    {
        ref_values.ref_pressure  = pressure_stats.getStats().mean;
        ref_values.ref_latitude  = gps_lat_stats.getStats().mean;
        ref_values.ref_longitude = gps_lon_stats.getStats().mean;
        ref_values.ref_accel_x   = accel_x_stats.getStats().mean;
        ref_values.ref_accel_y   = accel_y_stats.getStats().mean;
        ref_values.ref_accel_z   = accel_z_stats.getStats().mean;
        ref_values.ref_mag_x     = mag_x_stats.getStats().mean;
        ref_values.ref_mag_y     = mag_y_stats.getStats().mean;
        ref_values.ref_mag_z     = mag_z_stats.getStats().mean;
    }

    return ref_values;
}

}  // namespace DeathStackBoard