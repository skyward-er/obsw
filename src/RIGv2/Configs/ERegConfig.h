/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <RIGv2/Actuators/Actuators.h>
#include <algorithms/EReg/ERegPIDConfig.h>
#include <units/Frequency.h>

#include <chrono>

namespace RIGv2
{
namespace Config
{
namespace ERegFUEL
{

using namespace Boardcore::Units::Frequency;
using namespace std::chrono;

constexpr ServosList EREG_SERVO = ServosList::PRZ_FUEL_VALVE;

constexpr float TARGET_PRESSURE    = 58;  // [Bar]
constexpr Hertz UPDATE_RATE        = 100_hz;
constexpr int MEDIAN_SAMPLE_NUMBER = 10;

constexpr float UPDATE_RATE_SECONDS = 1 / UPDATE_RATE.value();  // [s]
constexpr float MAX_APERATURE       = 1.0f;
constexpr float MIN_APERATURE       = 0.0f;

const static Boardcore::ERegPIDConfig STABILIZING_CONFIG = {
    .KP = 0.07f,
    .KI = 0.02f,
    .KD = 0.0f,

    .Ts   = UPDATE_RATE_SECONDS,
    .uMin = MIN_APERATURE,
    .uMax = MAX_APERATURE,
};

const static Boardcore::ERegPIDConfig DISCHARGING_CONFIG = {
    .KP = 0.14,
    .KI = 0.03f,
    .KD = 0.01f,

    .Ts   = UPDATE_RATE_SECONDS,
    .uMin = MIN_APERATURE,
    .uMax = MAX_APERATURE,
};

}  // namespace ERegFUEL

namespace ERegOX
{

using namespace Boardcore::Units::Frequency;
using namespace std::chrono;

constexpr ServosList EREG_SERVO = ServosList::PRZ_OX_VALVE;

constexpr float TARGET_PRESSURE    = 58;  // [Bar]
constexpr Hertz UPDATE_RATE        = 100_hz;
constexpr int MEDIAN_SAMPLE_NUMBER = 10;

constexpr float UPDATE_RATE_SECONDS = 1 / UPDATE_RATE.value();  // [s]
constexpr float MAX_APERATURE       = 1.0f;
constexpr float MIN_APERATURE       = 0.0f;

const static Boardcore::ERegPIDConfig STABILIZING_CONFIG = {
    .KP = 0.07f,
    .KI = 0.02f,
    .KD = 0.0f,

    .Ts   = UPDATE_RATE_SECONDS,
    .uMin = MIN_APERATURE,
    .uMax = MAX_APERATURE,
};

const static Boardcore::ERegPIDConfig DISCHARGING_CONFIG = {
    .KP = 0.13f,
    .KI = 0.08f,
    .KD = 0.03f,

    .Ts   = UPDATE_RATE_SECONDS,
    .uMin = MIN_APERATURE,
    .uMax = MAX_APERATURE,
};

}  // namespace ERegOX
}  // namespace Config
}  // namespace RIGv2

