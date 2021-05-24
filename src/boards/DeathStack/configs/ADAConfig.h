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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Eigen/Dense>

#include "kalman/KalmanEigen.h"

namespace DeathStackBoard
{

namespace ADAConfigs
{

static const unsigned int ADA_UPDATE_PERIOD = 50;  // ms -> 20 hz

static const unsigned int ADA_PRIORITY = 2;  // high

// Number of consecutive samples with negative speed after which AD is triggered
constexpr unsigned int APOGEE_N_SAMPLES = 5;
// Number of consecutive samples with negative speed after which ABK are
// disabled
constexpr unsigned int ABK_DISABLE_N_SAMPLES = 5;
// Number of consecutive samples after which the main Deployment is triggered
constexpr unsigned int DEPLOYMENT_N_SAMPLES = 5;

// When the vertical speed is smaller than this value, apogee is detected.
// 0: Exact apogee
// > 0: Apogee detected ahead of time (while still going up)
constexpr float APOGEE_VERTICAL_SPEED_TARGET = 2.5;
// When the vertical speed is smaller than this value, aerobrakes are disabled.
constexpr float ABK_DISABLE_VERTICAL_SPEED_TARGET = 5.0;

// State timeouts
static const unsigned int TIMEOUT_ADA_SHADOW_MODE     = 7.5 * 1000;  // ms
static const unsigned int TIMEOUT_ADA_P_STABILIZATION = 5 * 1000;    // ms

// Number of samples used to calibrate the kalman initial state
static const unsigned int CALIBRATION_BARO_N_SAMPLES     = 50; // 1200;
static const unsigned int ACCELERATION_AVERAGE_N_SAMPLES = 25;

// Default reference values settings
// Standard atmosphere values @ Roccaraso
static const float DEFAULT_REFERENCE_TEMPERATURE = 278.920f;
static const float DEFAULT_REFERENCE_ALTITUDE    = 1420.0f;
static const float DEFAULT_REFERENCE_PRESSURE    = 85389.4f;

static const float DEFAULT_MSL_TEMPERATURE = 288.15f;
static const float DEFAULT_MSL_PRESSURE    = 101325.0f;

// Deployment altitude AGL
static const float DEFAULT_DEPLOYMENT_ALTITUDE = -100;

// Do cut the drogue above this altitude
static const float MAX_DEPLOYMENT_ALTITUDE_MSL = 1800;

// ------ Kalman parameters ------
static const float SAMPLING_PERIOD = 1 / 20.0f;  // In seconds

// Initialize the Kalman filter with a negative (pressure) acceleration in order
// to make it more respondive during the propulsive phase
static const float KALMAN_INITIAL_ACCELERATION = -500;

// kalman dimensions
static const uint8_t KALMAN_STATES_NUM  = 3;
static const uint8_t KALMAN_OUTPUTS_NUM = 1;

using MatrixNN = Matrix<float, KALMAN_STATES_NUM, KALMAN_STATES_NUM>;
using MatrixPN = Matrix<float, KALMAN_OUTPUTS_NUM, KALMAN_STATES_NUM>;
using MatrixNP = Matrix<float, KALMAN_STATES_NUM, KALMAN_OUTPUTS_NUM>;
using MatrixPP = Matrix<float, KALMAN_OUTPUTS_NUM, KALMAN_OUTPUTS_NUM>;
using CVectorN = Matrix<float, KALMAN_STATES_NUM, 1>;
using CVectorP = Matrix<float, KALMAN_OUTPUTS_NUM, 1>;

// clang-format off
static inline MatrixNN f_init()
{
    MatrixNN f;
    f << 1.0f, SAMPLING_PERIOD, 0.5f * SAMPLING_PERIOD * SAMPLING_PERIOD, 
         0.0f, 1.0f,            SAMPLING_PERIOD, 
         0.0f, 0.0f,            1.0f;

    return f;
}

static inline MatrixNN p_init()
{
    MatrixNN p;
    p << 0.1f, 0.0f, 0.0f, 
         0.0f, 0.0f, 0.0f, 
         0.0f, 0.0f, 0.0f;

    return p;
}

static inline MatrixNN q_init()
{
    MatrixNN q;
    q << 1.0f, 0.0f,  0.0f, 
         0.0f, 10.0f, 0.0f, 
         0.0f, 0.0f,  100.0f;

    return q;
}
// clang-format on

// kalman matrices
static const MatrixNN F_INIT = f_init();

// Output matrix
static const MatrixPN H_INIT{1.0f, 0.0f, 0.0f};

// Initial error covariance matrix
static const MatrixNN P_INIT = p_init();

// Model variance matrix
static const MatrixNN Q_INIT = q_init();

// Measurement variance
static const MatrixPP R_INIT{800.0f};

}  // namespace ADAConfigs

}  // namespace DeathStackBoard