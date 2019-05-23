/* Copyright (c) 2018,2019 Skyward Experimental Rocketry
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
#include <skyward-boardcore/libs/simple-template-matrix/matrix.h>

namespace DeathStackBoard
{
// TODO: Change with real values

// How many problematic gps samples to trigger an abort
constexpr unsigned int LHA_EGRESS_THRESHOLD = 10;

// Altitude at which the rogallo wing deploys even if there is no GPS fix or we
// are outside the launch safety area
constexpr unsigned int ROGALLO_UNCONDITIONAL_DPL_ALTITUDE = 100;

// Number of consecutive samples with negative speed after which AD is triggered
constexpr unsigned int APOGEE_N_SAMPLES = 5;

// State timeouts
static const unsigned int TIMEOUT_ADA_SHADOW_MODE = 6.5 * 1000;  // ms

// Number of samples used to calibrate the kalman initial state
static const unsigned int CALIBRATION_BARO_N_SAMPLES = 1200;

// Default reference values settings
// Standard atmosphere values @ Roccaraso
static const float DEFUALT_REFERENCE_TEMPERATURE = 279.700f;
static const float DEFUALT_REFERENCE_ALTITUDE    = 1300.0f;

static const float DEFUALT_MSL_TEMPERATURE = 288.15f;
static const float DEFUALT_MSL_PRESSURE    = 101325.0f;

// Deployment altitude AGL
// Set it under the ground level: don't deploy the Rogallo wing if we somehow
// forget to set the deployment altitude via telecommand
static const float DEFUALT_DEPLOYMENT_ALTITUDE = -100;

// ------ Kalman parameters ------
static const float SAMPLING_PERIOD = 1 / 20.0f;  // In seconds

// State matrix
// Note that sampling frequency is supposed to be constant and known at
// compile time. If this is not the case the matrix has to be updated at
// each iteration
static const MatrixBase<float, 3, 3> A_INIT(
    {1.0f, SAMPLING_PERIOD, 0.5f * SAMPLING_PERIOD* SAMPLING_PERIOD, 0.0f, 1.0f,
     SAMPLING_PERIOD, 0.0f, 0.0f, 1.0f});

// Output matrix
static const MatrixBase<float, 1, 3> C_INIT{1, 0, 0};

// Initial error covariance matrix
static const MatrixBase<float, 3, 3> P_INIT{0.1, 0, 0, 0, 0, 0, 0, 0, 0};

// Model variance matrix
static const MatrixBase<float, 3, 3> V1_INIT{1, 0, 0, 0, 10, 0, 0, 0, 10};

// Measurement variance
static const MatrixBase<float, 1, 1> V2_INIT{2500};
}  // namespace DeathStackBoard