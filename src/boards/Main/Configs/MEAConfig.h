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
constexpr float DEFAULT_INITIAL_ROCKET_MASS = 28.9286227299384f;

constexpr uint32_t SHADOW_MODE_TIMEOUT      = 900000;  // [ms]
constexpr float SHUTDOWN_THRESHOLD_ALTITUDE = 1000;    // [m]
constexpr unsigned int SHUTDOWN_N_SAMPLES   = 5;

// Pressure threshold after which the kalman is updated
constexpr float CC_PRESSURE_THRESHOLD = 1.f;

// Aerodynamics coefficients
constexpr float n000 = 0.554395329698886;
constexpr float n100 = -1.71019994711676;
constexpr float n200 = 8.05103009321528;
constexpr float n300 = -22.2129400612042;
constexpr float n400 = 34.6990670331566;
constexpr float n500 = -28.6219169089691;
constexpr float n600 = 9.73349655723146;

}  // namespace MEAConfig
}  // namespace Main