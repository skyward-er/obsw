/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Giovanni Annaloro
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

#include <algorithms/ZVK/ZVKConfig.h>
#include <common/ReferenceConfig.h>
#include <units/Frequency.h>

#include <Eigen/Dense>
#include <cmath>

namespace Main
{

namespace Config
{

namespace ZVK
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE         = 50_hz;
constexpr float UPDATE_RATE_SECONDS = 0.02;  // [s]
const Eigen::Vector3f onRampAttitude(133*pi/180, 85*pi/180, 0*pi/180); // z y x 

static const Boardcore::ZVKConfig CONFIG = {
    .T               = UPDATE_RATE_SECONDS,
    .SIGMA_ACC       = 4e-2,
    .SIGMA_BIAS_ACC  = 1e-5,
    .SIGMA_GYRO      = 5e-3,
    .SIGMA_GYRO_BIAS = 1e-5
};

}  // namespace ZVK

}  // namespace Config

}  // namespace Main
