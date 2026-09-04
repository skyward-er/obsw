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

#include <algorithms/Ereg/EregConfig.h>
#include <common/MavlinkHydra.h>
#include <units/Frequency.h>

#include <chrono>

namespace Motor
{
namespace Config
{
namespace EregFuel
{

/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace std::chrono;

constexpr ServosList EREG_SERVO    = ServosList::PRZ_FUEL_VALVE;
constexpr ServosList VENTING_SERVO = ServosList::FUEL_VENTING_VALVE;
constexpr auto VENTING_TIME        = 5000ms;

constexpr float PRESSURE_THRESHOLD = 0.01f;  // [Bar]

constexpr float TARGET_PRESSURE      = 55.0f;  // [Bar]
constexpr float PILOT_FLAME_INTEGRAL = 0.0502f;
constexpr float RAMPUP_INTEGRAL      = 0.2511f;
constexpr Hertz UPDATE_RATE          = 100_hz;
constexpr int FILTER_SAMPLES         = 3;

constexpr float UPDATE_RATE_SECONDS = 1 / UPDATE_RATE.value();  // [s]

const static Boardcore::EregPIDConfig STABILIZING_CONFIG = {
    .KP = 0.005f,
    .KI = 0.0f,
    .KD = 0.0f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::EregPIDConfig DISCHARGING_CONFIG = {
    .KP = 1.4f,
    .KI = 0.2f,
    .KD = 0.02f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::EregValveInfo VALVE_INFO = {
    .minServoPosition = 0.1486f,
    .minValveAngle    = 22.8f,
    .maxCv            = 0.912291f,

    .polyValveCoeff = {11.5576f, -27.1038f, 23.5596f, -9.4317f, 2.3962f,
                       0.0032f},

    .polyServoCoeff = {523.9735e-012f, -109.5398e-009f, 9.3893e-006f,
                       -357.9286e-006f, 14.5807e-003f, 3.7033e-003f},
};

}  // namespace EregFuel

namespace EregOx
{

using namespace Boardcore::Units::Frequency;
using namespace std::chrono;

constexpr ServosList EREG_SERVO    = ServosList::PRZ_OX_VALVE;
constexpr ServosList VENTING_SERVO = ServosList::OX_VENTING_VALVE;
constexpr auto VENTING_TIME        = 5000ms;

constexpr float PRESSURE_THRESHOLD = 0.01f;  // [Bar]

constexpr float TARGET_PRESSURE      = 53.0f;  // [Bar]
constexpr float PILOT_FLAME_INTEGRAL = 0.1844f;
constexpr float RAMPUP_INTEGRAL      = 1.4346f;
constexpr Hertz UPDATE_RATE          = 100_hz;
constexpr int FILTER_SAMPLES         = 3;

constexpr float UPDATE_RATE_SECONDS = 1 / UPDATE_RATE.value();  // [s]

const static Boardcore::EregPIDConfig STABILIZING_CONFIG = {
    .KP = 0.03f,
    .KI = 0.0f,
    .KD = 0.0f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::EregPIDConfig DISCHARGING_CONFIG = {
    .KP = 1.3f,
    .KI = 0.4f,
    .KD = 0.06f,

    .Ts = UPDATE_RATE_SECONDS,
};

const static Boardcore::EregValveInfo VALVE_INFO = {
    .minServoPosition = 0.1580f,
    .minValveAngle    = 23.6f,
    .maxCv            = 0.981898f,

    .polyValveCoeff = {9.7769f, -23.6167f, 21.5392f, -9.2101f, 2.4963f,
                       0.0036f},

    .polyServoCoeff = {732.2123e-012f, -126.0129e-009f, 8.7960e-006f,
                       -282.0294e-006f, 13.5809e-003f, 7.3421e-003f},
};

}  // namespace EregOx
}  // namespace Config
}  // namespace Motor

