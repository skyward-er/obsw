/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <cmath>

namespace Ciuti
{

namespace UprightDetectorConfig
{

constexpr int ALGO_FREQUENCY        = 50;
constexpr int DETECT_TIME           = 120;
constexpr int MEAN_SAMPLES          = 50;
constexpr float DETECT_ANGLE        = 83;
constexpr float THRESHOLD_TOLERANCE = 0.2;

constexpr int ALGO_PERIOD    = 1000 / 50;
constexpr int DETECT_SAMPLES = DETECT_TIME * ALGO_FREQUENCY;

constexpr float THRESHOLD =
    std::sin(DETECT_ANGLE * M_PI / 180.0) * (1.0 - THRESHOLD_TOLERANCE);

}  // namespace UprightDetectorConfig

}  // namespace Ciuti
