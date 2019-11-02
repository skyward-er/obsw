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
    ADA(ADASetupData setup_data);
    ~ADA();

    void updateBaro(float pressure);
    void updateAcc(float ax);

    float getAltitude();
    float getVerticalSpeed();

    KalmanState getKalmanState();

private:
    Kalman<3, 1> filter;      // Filter object
    Kalman<3, 2> filter_acc;  // Filter with accelerometer

    // Stats for acceleration averaging
    Stats acc_stats;

    // References for pressure to altitude conversion
    ReferenceValues ref_values;
};
}  // namespace DeathStackBoard