/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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

#include <units/Frequency.h>

#include <chrono>
#include <cstdint>

namespace Motor
{
namespace Config
{

#define VALVE_THRESHOLD(name)

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto VALVE_OPENING_THRESHOLD_MAIN_OX      = 60.0f;  // [percent]
constexpr auto VALVE_OPENING_THRESHOLD_MAIN_FUEL    = 60.0f;
constexpr auto VALVE_OPENING_THRESHOLD_PRZ_OX       = 60.0f;
constexpr auto VALVE_OPENING_THRESHOLD_PRZ_FUEL     = 60.0f;
constexpr auto VALVE_OPENING_THRESHOLD_OX_VENTING   = 60.0f;
constexpr auto VALVE_OPENING_THRESHOLD_FUEL_VENTING = 60.0f;

constexpr auto VALVE_CLOSED_THRESHOLD_MAIN_OX      = 10.0f;
constexpr auto VALVE_CLOSED_THRESHOLD_MAIN_FUEL    = 10.0f;
constexpr auto VALVE_CLOSED_THRESHOLD_PRZ_OX       = 10.0f;
constexpr auto VALVE_CLOSED_THRESHOLD_PRZ_FUEL     = 10.0f;
constexpr auto VALVE_CLOSED_THRESHOLD_OX_VENTING   = 10.0f;
constexpr auto VALVE_CLOSED_THRESHOLD_FUEL_VENTING = 10.0f;

constexpr auto VALVE_WIGGLE_DELAY  = 1000;  // [ms]
constexpr auto VALVE_CLOSING_DELAY = 6500;  // [ms]

namespace DepressurizationConfig
{
constexpr auto OX_VENTING_TIMEOUT          = 60s;
constexpr auto DEPRESSURIZATION_CHECK_RATE = 10_hz;
constexpr float OX_PRESSURE_THRESHOLD      = 10.0f;  // [bar]
constexpr auto OX_HYSTERESIS               = 1s;

constexpr float PRZ_OX_APERTURE           = .4f;
constexpr float PRZ_OX_PRESSURE_THRESHOLD = 20.0f;
constexpr auto PRZ_OX_HYSTERESIS          = 1s;
constexpr auto PRZ_OX_TIMEOUT             = 60s;
constexpr auto PRZ_FUEL_TIMEOUT           = 30s;
}  // namespace DepressurizationConfig
}  // namespace Config
}  // namespace Motor
