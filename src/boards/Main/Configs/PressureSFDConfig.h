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

constexpr FeaturesArr BETA  = {0.0538048411163895,  1.21829189675103,
                               -0.0135044925652423, -2.46984492432602,
                               -1.7469906902922,    0.236220477009252};
constexpr FeaturesArr MU    = {1306.72004416967, 0.351190954475099,
                               1.99867039117899, -0.0185223927903996,
                               7.45589640537251, 12.630364102307};
constexpr FeaturesArr SIGMA = {1055.13277492127, 0.0457124377157068,
                               1.83336893903227, 0.0874316265177542,
                               1.19779014092547, 0.999724384795152};
constexpr float BIAS        = -0.945291058126819;
constexpr float SCALE       = 3.25293089128877;

constexpr SVMConf SVM_CONF = {BETA, MU, SIGMA, BIAS, SCALE};
}  // namespace Ascent

// Descent SFD model
namespace Descent
{
using FeaturesArr = std::array<float, Boardcore::SFDDescent::NUM_FEATURES>;
using SVMConf = Boardcore::SVM<Boardcore::SFDDescent::NUM_FEATURES>::SVMConfig;

constexpr FeaturesArr BETA  = {-1.45789357962751, 0.149212230274532,
                               0.543310785123195, 0.36657833324541,
                               -0.033476999690174};
constexpr FeaturesArr MU    = {1.69681676472682, 261.15944469953,
                               0.0114112487465149, -0.0168260044537472,
                               2.1355858817835};
constexpr FeaturesArr SIGMA = {0.20823828849293, 356.048735537809,
                               0.156943288015148, 0.480743200014727,
                               0.912588579044107};
constexpr float BIAS        = -4.55617842982864;
constexpr float SCALE       = 0.775015678995177;

constexpr SVMConf SVM_CONF = {BETA, MU, SIGMA, BIAS, SCALE};
}  // namespace Descent

// LowPass filter
namespace LowPass
{
constexpr float CUTOFF_FREQ = 500.0;  // [Hz]
constexpr float GAIN        = 500.0;
constexpr float LAMBDA      = 0.000045399929762484854;
}  // namespace LowPass

}  // namespace PressureSFDConfig
}  // namespace Main
