/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
namespace MEAConfig
{
constexpr uint32_t UPDATE_PERIOD = 20;  // [ms] -> 50Hz

constexpr float SENSOR_NOISE_VARIANCE       = 0.36f;
constexpr float MODEL_NOISE_VARIANCE        = 0.1f;
constexpr float DEFAULT_INITIAL_ROCKET_MASS = 35.01f;

constexpr uint32_t SHADOW_MODE_TIMEOUT      = 7000;  // [ms]
constexpr float SHUTDOWN_THRESHOLD_ALTITUDE = 3200;  // [m]
constexpr unsigned int SHUTDOWN_N_SAMPLES   = 5;

// Pressure threshold after which the kalman is updated
constexpr float CC_PRESSURE_THRESHOLD = 1.f;

// Aerodynamics coefficients
constexpr float n000 = 0.596535425207973;
constexpr float n100 = -1.81429600946981;
constexpr float n200 = 8.47683559348987;
constexpr float n300 = -23.1759370919254;
constexpr float n400 = 35.8276525337534;
constexpr float n500 = -29.2336913633527;
constexpr float n600 = 9.84223649075812;

}  // namespace MEAConfig
}  // namespace Main