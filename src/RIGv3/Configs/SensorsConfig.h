/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <units/Frequency.h>

#include <chrono>
#include <cstdint>

namespace RIGv3
{

namespace Config
{

namespace Sensors
{

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

namespace ADS131M08
{
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_256;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

constexpr Hertz PERIOD = 1000_hz;
}  // namespace ADS131M08

namespace ADC_1
{
constexpr bool ENABLED = true;

}  // namespace ADC_1

namespace InternalADC
{
constexpr bool ENABLED = true;
constexpr Hertz PERIOD = 10_hz;
}  // namespace InternalADC

}  // namespace Sensors
}  // namespace Config
}  // namespace RIGv3
