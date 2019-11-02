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

#pragma once

#include <Common.h>
#include <Debug.h>
#include <math/Stats.h>
#include <miosix.h>
#include "ADAStatus.h"

namespace DeathStackBoard
{
class ADACalibrator
{
private:
    /* --- CALIBRATION --- */
    Stats pressure_stats;  // Computes mean std dev etc for calibration of
                           // pressure conversion

    ADASetupData setup_data;

public:
    /* --- CALIBRATION --- */
    ADASetupData getSetupData() { return setup_data; };
    bool calibIsComplete();
    void addBaroSample(float p);
    void resetBaro();

    /* --- TC ---*/
    /**
     * Sets the reference temperature to be used to calibrate the altimeter
     * @param ref_temp Reference temperature in degrees Celsisus
     */
    void setReferenceTemperature(float ref_temp);

    /**
     * Sets the reference altitude to be used to calibrate the altimeter
     * @param ref_alt Reference altitude in meters above mean sea level
     */
    void setReferenceAltitude(float ref_alt);
};
}  // namespace DeathStackBoard