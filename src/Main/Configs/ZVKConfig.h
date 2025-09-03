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

namespace Main
{

namespace Config
{

namespace ZVK
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE         = 50_hz;
Eigen::Vector3f initialAttitude = {0,0,0};

//TO DO : ASK GNC ACTUAL PARAMETERS
static const Boardcore::ZVKConfig CONFIG = {
    .T               = UPDATE_RATE_SECONDS,
    .TUNE_PARAM_mu   = 0,    
    .TUNE_PARAM_Re   = 0,    
    .TUNE_PARAM_J2   = 0,
    .SIGMA_GYRO      = 0,   
    .SIGMA_GYRO_BIAS = 0,
    .SIGMA_ACC       = 0,  
    .SIGMA_BIAS_ACC  = 0,   
    .SIGMA_MAG       = 0,        
    .BIAS_ACC        = 0,         
    .BIAS_GYRO       = 0,    
    .VEL_UNCERTAINTY = 0,
    .POS_UNCERTAINTY = 0,
    .NED_MAG         = Common::ReferenceConfig::nedMag};

}  // namespace ZVK

}  // namespace Config

}  // namespace Main
