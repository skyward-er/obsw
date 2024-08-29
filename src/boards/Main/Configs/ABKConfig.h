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

#include <algorithms/AirBrakes/AirBrakesInterp.h>
#include <units/Frequency.h>

#include <cstdint>

namespace Main
{

namespace Config
{

namespace ABK
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE = 10_hz;

constexpr unsigned int SHADOW_MODE_TIMEOUT = 500;  // [ms]

static const Boardcore::AirBrakesInterpConfig CONFIG = {
    .FILTER_MINIMUM_ALTITUDE = 1000,
    .FILTER_MAXIMUM_ALTITUDE = 3000,
    .STARTING_FILTER_VALUE   = 0.9f,
    .ABK_CRITICAL_ALTITUDE   = 2990,
    .DZ                      = 10,
    .INITIAL_MASS            = 26,
    .DM                      = 0.4f,
    .N_FORWARD               = 0};

}  // namespace ABK

}  // namespace Config

}  // namespace Main