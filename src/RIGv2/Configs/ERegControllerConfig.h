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
#include <algorithms/EReg/ERegConfig.h>
#include <units/Frequency.h>

#include <chrono>

namespace RIGv2
{
namespace Config
{
namespace ERegFuel
{

using namespace Boardcore::Units::Frequency;
using namespace std::chrono;

// why is this not static constexpr?
constexpr ServosList EREG_SERVO = ServosList::PRZ_FUEL_VALVE;

constexpr float PRESSURE_THRESHOLD = 0.01f;  // [Bar]

constexpr float TARGET_PRESSURE      = 58;  // [Bar]
constexpr float PILOT_FLAME_INTEGRAL = 0.0f;
constexpr float RAMPUP_INTEGRAL      = 0.0f;
constexpr Hertz UPDATE_RATE          = 100_hz;
constexpr int FILTER_SAMPLES         = 3;

constexpr float UPDATE_RATE_SECONDS = 1 / UPDATE_RATE.value();  // [s]

const static Boardcore::ERegPIDConfig STABILIZING_CONFIG = {
    .KP = 0.004f,
    .KI = 0.0f,
    .KD = 0.0f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::ERegPIDConfig DISCHARGING_CONFIG = {
    .KP = 0.13f,
    .KI = 0.0f,
    .KD = 0.0f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::ERegValveInfo VALVE_INFO = {
    .minServoPosition = 0.1012f,
    .minValveAngle    = 12,
    .maxCv            = 0.912291f,

    .polyValveCoeff = {11.5576f, -27.1038f, 23.5596f, -9.4317f, 2.3962f,
                       0.0032f},

    .polyServoCoeff = {0.0f, 0.0f, 1.3302e-6f, -169.3787e-6f, 18.6759e-3f,
                       11.8442e-3f},
};

}  // namespace ERegFuel

namespace ERegOx
{

using namespace Boardcore::Units::Frequency;
using namespace std::chrono;

constexpr ServosList EREG_SERVO = ServosList::PRZ_OX_VALVE;

constexpr float PRESSURE_THRESHOLD = 0.01f;  // [Bar]

constexpr float TARGET_PRESSURE      = 58;  // [Bar]
constexpr float PILOT_FLAME_INTEGRAL = 0.0f;
constexpr float RAMPUP_INTEGRAL      = 0.0f;
constexpr Hertz UPDATE_RATE          = 100_hz;
constexpr int FILTER_SAMPLES         = 3;

constexpr float UPDATE_RATE_SECONDS = 1 / UPDATE_RATE.value();  // [s]

const static Boardcore::ERegPIDConfig STABILIZING_CONFIG = {
    .KP = 0.01f,
    .KI = 0.0f,
    .KD = 0.0f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::ERegPIDConfig DISCHARGING_CONFIG = {
    .KP = 0.26f,
    .KI = 0.0f,
    .KD = 0.0f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::ERegValveInfo VALVE_INFO = {
    .minServoPosition = 0.1163f,
    .minValveAngle    = 16,
    .maxCv            = 0.981898f,

    .polyValveCoeff = {9.7769f, -23.6167f, 21.5392f, -9.2101f, 2.4963f,
                       0.0036f},

    .polyServoCoeff = {1.3476e-9f, -258.0271e-9f, 18.2523e-6f, -577.5516e-6f,
                       16.5135e-3f, 2.7180e-3f},
};

}  // namespace ERegOx
}  // namespace Config
}  // namespace RIGv2

