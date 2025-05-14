/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Mandelli, Davide Basso
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

#include <miosix.h>
#include <units/Time.h>

namespace Parafoil
{

namespace Config
{

/**
 * Configuration for the Wing Estimation Scheme (WES) algorithm.
 */
namespace WES
{

/* linter-off */ using namespace Boardcore::Units::Time;

constexpr auto CALIBRATE = false;  // FIXME: Wind Estimation implementation is
                                   // not finished yet

constexpr auto CALIBRATION_TIMEOUT = 5_s;  // time needed for the first loop
constexpr auto ROTATION_PERIOD     = 10_s;
constexpr auto CALIBRATION_SAMPLE_NUMBER =
    20;  // number of samples to take in the first loop
constexpr auto CALIBRATION_UPDATE_PERIOD =
    CALIBRATION_TIMEOUT / CALIBRATION_SAMPLE_NUMBER;
constexpr auto PREDICTION_UPDATE_PERIOD = 100_ms;  // update period of WES[ms]

}  // namespace WES

}  // namespace Config

}  // namespace Parafoil
