/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

namespace Parafoil
{

namespace WESConfig
{

constexpr uint32_t WES_CALIBRATION_TIMEOUT =
    5 * 1000;  // time needed for the first loop [ms]
constexpr uint32_t WES_TIMEOUT =
    2 * WES_CALIBRATION_TIMEOUT;  // time needed for the second loop

constexpr int WES_CALIBRATION_SAMPLE_NUMBER =
    20;  // number to sample to take in the first loop
constexpr uint32_t WES_CALIBRATION_UPDATE_PERIOD =
    WES_CALIBRATION_TIMEOUT / WES_CALIBRATION_SAMPLE_NUMBER;
constexpr uint32_t WES_PREDICTION_UPDATE_PERIOD =
    100;  // update period of WES[ms]

}  // namespace WESConfig

}  // namespace Parafoil
