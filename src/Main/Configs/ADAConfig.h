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

#include <units/Frequency.h>

#include <chrono>
#include <cstdint>

namespace Main
{

namespace Config
{

namespace ADA
{

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE         = 50_hz;
constexpr float UPDATE_RATE_SECONDS = 0.02;  // [s]

constexpr float APOGEE_VERTICAL_SPEED_TARGET = 2.5;  // [m/s]
constexpr unsigned int APOGEE_N_SAMPLES      = 5;

constexpr float DEPLOYMENT_ALTITUDE_TARGET  = 370;  // [m]
constexpr unsigned int DEPLOYMENT_N_SAMPLES = 5;

#ifdef ROCCARASO
constexpr auto SHADOW_MODE_TIMEOUT = 10s;

#else  // EUROC
constexpr auto SHADOW_MODE_TIMEOUT = 12s;

#ifndef EUROC
#warning "ADAConfig: no mission specified, using EUROC"
#endif
#endif

}  // namespace ADA

}  // namespace Config

}  // namespace Main
