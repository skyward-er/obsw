/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <stdint.h>

namespace Main
{

namespace ADAConfig
{

static constexpr uint32_t UPDATE_PERIOD = 20;                  // 50 hz
static const float SAMPLING_PERIOD = UPDATE_PERIOD / 1000.0f;  // in seconds

// Initialize the Kalman filter with a negative (pressure) acceleration in order
// to make it more responsive during the propulsive phase
static const float KALMAN_INITIAL_ACCELERATION = -500;

#ifdef EUROC
static constexpr int SHADOW_MODE_TIMEOUT = 16 * 1000;  // [ms]
#else
static constexpr int SHADOW_MODE_TIMEOUT      = 8 * 1000;  // [ms]
#endif

static constexpr int PRES_STAB_TIMEOUT = 5 * 1000;  // [ms]

// Default reference values settings
#ifdef EUROC
static const float DEFAULT_REFERENCE_ALTITUDE = 160.0f;
static const float DEFAULT_REFERENCE_PRESSURE = 100022.4f;
#else
static const float DEFAULT_REFERENCE_ALTITUDE = 1420.0f;
static const float DEFAULT_REFERENCE_PRESSURE = 85389.4f;
#endif

static const float DEFAULT_REFERENCE_TEMPERATURE = 288.15f;

// When the vertical speed is smaller than this value, apogee is detected.
// If equal to 0 ->     Exact apogee
// If greater than 0 -> Apogee detected ahead of time (while still going up)
constexpr float APOGEE_VERTICAL_SPEED_TARGET = 2.5;  // [m/s]

// Number of consecutive samples after which apogee is triggered.
constexpr unsigned int APOGEE_N_SAMPLES = 5;

// Number of consecutive samples after which airbrakes are disabled.
constexpr unsigned int ABK_DISABLE_N_SAMPLES = 5;

// Number of consecutive samples after which the main deployment is triggered.
constexpr unsigned int DEPLOYMENT_N_SAMPLES = 5;

// Number of consecutive samples after which the landing is triggered.
constexpr unsigned int LANDING_N_SAMPLES = 5;

// Deployment altitude above ground level
static const float DEFAULT_DEPLOYMENT_ALTITUDE = 350;

// Vertical speed magnitude under which the landing is triggered
constexpr float LANDING_VERTICAL_SPEED_MAG_TARGET = 2.5;  // [m/s]

}  // namespace ADAConfig

}  // namespace Main
