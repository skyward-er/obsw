/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <algorithms/SFD/SFDAscent.h>
#include <algorithms/SFD/SFDDescent.h>

#include <array>

namespace Main
{
namespace PressureSFDConfig
{

// Ascent SFD model
namespace Ascent
{
using FeaturesArr = std::array<float, Boardcore::SFDAscent::NUM_FEATURES>;
using SVMConf = Boardcore::SVM<Boardcore::SFDAscent::NUM_FEATURES>::SVMConfig;

constexpr FeaturesArr BETA  = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr FeaturesArr MU    = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr FeaturesArr SIGMA = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr float BIAS        = 0.0;
constexpr float SCALE       = 0.0;

constexpr SVMConf SVM_CONF = {BETA, MU, SIGMA, BIAS, SCALE};
}  // namespace Ascent

// Descent SFD model
namespace Descent
{
using FeaturesArr = std::array<float, Boardcore::SFDDescent::NUM_FEATURES>;
using SVMConf = Boardcore::SVM<Boardcore::SFDDescent::NUM_FEATURES>::SVMConfig;

constexpr FeaturesArr BETA  = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr FeaturesArr MU    = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr FeaturesArr SIGMA = {0.0, 0.0, 0.0, 0.0, 0.0};
constexpr float BIAS        = 0.0;
constexpr float SCALE       = 0.0;

constexpr SVMConf SVM_CONF = {BETA, MU, SIGMA, BIAS, SCALE};
}  // namespace Descent

// LowPass filter
namespace LowPass
{
constexpr float CUTOFF_FREQ = 0.0;
constexpr float GAIN        = 0.0;
constexpr float LAMBDA      = 0.0;
}  // namespace LowPass

}  // namespace PressureSFDConfig
}  // namespace Main
