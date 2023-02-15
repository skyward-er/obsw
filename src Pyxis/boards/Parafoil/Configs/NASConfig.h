/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <algorithms/NAS/NASConfig.h>

namespace Parafoil
{

namespace NASConfig
{

constexpr uint32_t UPDATE_PERIOD = 50;  // 50 hz

// Magnetic field in Milan
Eigen::Vector3f nedMag(0.4747, 0.0276, 0.8797);

static const Boardcore::NASConfig config = {
    1.0f / UPDATE_PERIOD,  // T
    0.0001f,               // SIGMA_BETA
    0.3f,                  // SIGMA_W
    0.1f,                  // SIGMA_ACC
    0.1f,                  // SIGMA_MAG
    10.0f,                 // SIGMA_GPS
    4.3f,                  // SIGMA_BAR
    10.0f,                 // SIGMA_POS
    10.0f,                 // SIGMA_VEL
    10.0f,                 // SIGMA_PITOT
    1.0f,                  // P_POS
    10.0f,                 // P_POS_VERTICAL
    1.0f,                  // P_VEL
    10.0f,                 // P_VEL_VERTICAL
    0.01f,                 // P_ATT
    0.01f,                 // P_BIAS
    6.0f,                  // SATS_NUM
    nedMag                 // NED_MAG
};

}  // namespace NASConfig

}  // namespace Parafoil
