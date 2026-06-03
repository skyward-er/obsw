/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <units/Frequency.h>

#include <chrono>

namespace Main
{

namespace Config
{

namespace SDA
{

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE = 50_hz;

#ifdef ROCCARASO
constexpr auto SHADOW_MODE_TIMEOUT          = 2200ms;
constexpr float DEFAULT_INITIAL_ROCKET_MASS = 35.5f;  // [kg]
constexpr float SHUTDOWN_APOGEE_TARGET      = 1050;   // agl [m]

#else  // EUROC
constexpr auto SHADOW_MODE_TIMEOUT          = 4000ms;
constexpr float DEFAULT_INITIAL_ROCKET_MASS = 35.5f;  // [kg]
constexpr float SHUTDOWN_APOGEE_TARGET      = 3000;   // agl [m]

#ifndef EUROC
#warning "SDAConfig: no mission specified, using EUROC"
#endif
#endif

/* constexpr float CD_CORRECTION_FACTOR = 1.f;

constexpr unsigned int SHUTDOWN_N_SAMPLES = 5;

constexpr float SENSOR_NOISE_VARIANCE = 0.36f;
constexpr float MODEL_NOISE_VARIANCE  = 1.0f;

// Pressure threshold after which the kalman is updated
constexpr float CC_PRESSURE_THRESHOLD = 1.0f;  // [bar] */
}  // namespace SDA

}  // namespace Config

}  // namespace Main
