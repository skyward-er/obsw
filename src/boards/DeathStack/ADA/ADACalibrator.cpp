/* Copyright (c) 2018,2019 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Erbetta
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

#include "ADACalibrator.h"

namespace DeathStackBoard
{

void ADACalibrator::addBaroSample(float p)
{
    pressure_stats.add(p);
    setup_data.pressure_stats_results = pressure_stats.getStats();
}

bool ADACalibrator::calibIsComplete()
{
    return setup_data.pressure_stats_results.nSamples >= CALIBRATION_BARO_N_SAMPLES &&
           setup_data.ref_alt_set && setup_data.ref_temp_set;
}

void ADACalibrator::resetStats()
{
    pressure_stats.reset();
    setup_data.pressure_stats_results = pressure_stats.getStats();
}

void ADACalibrator::setReferenceTemperature(float ref_temp)
{
    float pressure_ref = ref_temp + 273.15; // Celsius to Kelvin

    // Sanity check: Obey to the laws of thermodynamics
    if (pressure_ref > 0)
    {
        setup_data.ref_temp     = ref_temp + 273.15;
        setup_data.ref_temp_set = true;
        TRACE("[ADA] Reference temperature set to %.3f K\n", setup_data.ref_temp);
    }
}

void ADACalibrator::setReferenceAltitude(float ref_alt)
{
    setup_data.ref_alt     = ref_alt;
    setup_data.ref_alt_set = true;
    TRACE("[ADA] Reference altitude set to %.3f m\n", ref_alt);
}


}