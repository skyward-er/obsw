/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <algorithms/MEA/MEA.h>
#include <units/Frequency.h>

namespace Main
{

namespace Config
{

namespace MEA
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE = 50_hz;

#ifdef ROCCARASO
// TODO: update for roccaraso flight
constexpr unsigned int SHADOW_MODE_TIMEOUT  = 2500;    // [ms]
constexpr float DEFAULT_INITIAL_ROCKET_MASS = 31.69f;  // [kg]
constexpr float SHUTDOWN_APOGEE_TARGET      = 1000;    // agl [m]
#else
#ifndef EUROC
#warning "MISSION NOT DEFINED: Using EUROC"
#endif
constexpr unsigned int SHADOW_MODE_TIMEOUT  = 4500;    // [ms]
constexpr float DEFAULT_INITIAL_ROCKET_MASS = 33.69f;  // [kg]
constexpr float SHUTDOWN_APOGEE_TARGET      = 3200;    // agl [m]
#endif

constexpr float CD_CORRECTION_FACTOR =
    2.5f;  // Completely random number (for now... i hope)

constexpr unsigned int SHUTDOWN_N_SAMPLES = 5;

constexpr float SENSOR_NOISE_VARIANCE = 3.9681e-06f;
constexpr float MODEL_NOISE_VARIANCE  = 1.0f;

// Pressure threshold after which the kalman is updated
constexpr float CC_PRESSURE_THRESHOLD = 1.f;  // [bar]

constexpr Boardcore::Aeroutils::AerodynamicCoeff AERO_COEFF = {
    .n000 = 0.596535425207973f,
    .n100 = -1.81429600946981f,
    .n200 = 8.47683559348987f,
    .n300 = -23.1759370919254f,
    .n400 = 35.8276525337534f,
    .n500 = -29.2336913633527f,
    .n600 = 9.84223649075812f};

}  // namespace MEA

}  // namespace Config

}  // namespace Main