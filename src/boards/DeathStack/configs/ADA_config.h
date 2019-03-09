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
#include <kalman/Matrix.hpp>

namespace DeathStackBoard
{
// TODO: Change with real values

// State timeouts
static const unsigned int TIMEOUT_MS_CALIBRATION      = 15 * 1000;
static const unsigned int CALIBRATION_N_SAMPLES       = 5000;

// Kalman parameters
static const float P_DATA[9] = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};    // Initial error covariance matrix
static const float R_DATA[1] = {10};                                 // Measurement variance  
static const float Q_DATA[9] = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01}; // Model variance matrix
static const float H_DATA[3] = {1, 0, 0};
static const float SAMPLING_PERIOD = 0.01; // In seconds

static const Matrix P_INIT{3, 3, P_DATA};
static const Matrix R_INIT{1, 1, R_DATA};
static const Matrix Q_INIT{3, 3, Q_DATA};
static const Matrix H_INIT{1, 3, H_DATA};
}