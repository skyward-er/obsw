/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Matteo Pignataro
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
constexpr uint32_t UPDATE_PERIOD = 20;                       // [ms] 50Hz
constexpr float SAMPLING_PERIOD  = UPDATE_PERIOD / 1000.0f;  // [seconds]

// Calibration constants
constexpr int CALIBRATION_SAMPLES_COUNT = 20;
constexpr int CALIBRATION_SLEEP_TIME    = 100;  // [ms]

// ADA shadow mode time, during which the ADA algorithm cannot trigger apogees
constexpr uint32_t SHADOW_MODE_TIMEOUT = 10000;  // [ms]

// When the vertical speed is smaller than this value, apogee is detected.
// If equal to 0 ->     Exact apogee
// If greater than 0 -> Apogee detected ahead of time (while still going up)
constexpr float APOGEE_VERTICAL_SPEED_TARGET = 2.5;  // [m/s]

// Number of consecutive samples after which apogee is triggered.
constexpr unsigned int APOGEE_N_SAMPLES = 5;

}  // namespace ADAConfig
}  // namespace Main