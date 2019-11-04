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
#include <utils/aero/AeroUtils.h>
namespace DeathStackBoard
{

void ADACalibrator::addBaroSample(float p) { pressure_stats.add(p); }

bool ADACalibrator::calibIsComplete()
{
    // Calibration is complete if enough samples were collected and reference
    // altitude and temperature were set
    return pressure_stats.getStats().nSamples >= CALIBRATION_BARO_N_SAMPLES &&
           ref_alt_set && ref_temp_set;
}

void ADACalibrator::resetBaro() { pressure_stats.reset(); }

void ADACalibrator::setReferenceTemperature(float ref_temp)
{
    float temperature_ref = ref_temp + 273.15;  // Celsius to Kelvin

    // Sanity check: Obey to the laws of thermodynamics
    if (temperature_ref > 0)
    {
        ref_values.ref_temperature = temperature_ref;
        ref_temp_set               = true;
        TRACE("[ADA] Reference temperature set to %.3f K\n", temperature_ref);
    }
}

void ADACalibrator::setReferenceAltitude(float ref_alt)
{
    ref_values.ref_altitude = ref_alt;
    ref_alt_set             = true;
    TRACE("[ADA] Reference altitude set to %.3f m\n", ref_alt);
}

ReferenceValues ADACalibrator::getReferenceValues()
{
    // If calibration is not compete use default values
    if (calibIsComplete())
    {
        ref_values.ref_pressure = pressure_stats.getStats().mean;
    }
    else
    {
        ref_values.ref_pressure = DEFAULT_REFERENCE_PRESSURE;
    }

    // Calculate MSL values for altitude Pa/m conversion
    ref_values.msl_pressure = aeroutils::mslPressure(ref_values.ref_pressure,
                                                     ref_values.ref_temperature,
                                                     ref_values.ref_altitude);

    ref_values.msl_temperature = aeroutils::mslTemperature(
        ref_values.ref_temperature, ref_values.ref_altitude);

    return ref_values;
}
}  // namespace DeathStackBoard