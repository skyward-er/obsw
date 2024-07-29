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

#include <algorithms/NAS/NASConfig.h>
#include <units/Frequency.h>
#include <common/ReferenceConfig.h>

namespace Main 
{

namespace Config
{

namespace NAS
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz SAMPLE_RATE = 50_hz;

static const Boardcore::NASConfig CONFIG = {
    .T = 0.02, // [s]
    .SIGMA_BETA = 0.0001,
    .SIGMA_W = 0.3, 
    .SIGMA_ACC = 0.1, 
    .SIGMA_MAG = 0.1, 
    .SIGMA_GPS = 10.0, 
    .SIGMA_BAR = 4.3, 
    .SIGMA_POS = 10.0, 
    .SIGMA_VEL = 10.0, 
    .SIGMA_PITOT = 10.0, 
    .P_POS = 1.0, 
    .P_POS_VERTICAL = 10.0, 
    .P_VEL = 1.0, 
    .P_VEL_VERTICAL = 10.0, 
    .P_ATT = 0.01, 
    .P_BIAS = 0.01, 
    .SATS_NUM = 6.0,
    .NED_MAG = Common::ReferenceConfig::nedMag
};

}

}

}