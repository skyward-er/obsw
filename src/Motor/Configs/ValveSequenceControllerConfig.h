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

constexpr auto VALVE_OPENING_THRESHOLD_MAIN_OX      = 60;
constexpr auto VALVE_OPENING_THRESHOLD_MAIN_FUEL    = 60;
constexpr auto VALVE_OPENING_THRESHOLD_PRZ_OX       = 60;
constexpr auto VALVE_OPENING_THRESHOLD_PRZ_FUEL     = 60;
constexpr auto VALVE_OPENING_THRESHOLD_OX_VENTING   = 10;
constexpr auto VALVE_OPENING_THRESHOLD_FUEL_VENTING = 10;

constexpr auto VALVE_CLOSED_THRESHOLD_MAIN_OX      = 10;
constexpr auto VALVE_CLOSED_THRESHOLD_MAIN_FUEL    = 10;
constexpr auto VALVE_CLOSED_THRESHOLD_PRZ_OX       = 10;
constexpr auto VALVE_CLOSED_THRESHOLD_PRZ_FUEL     = 10;
constexpr auto VALVE_CLOSED_THRESHOLD_OX_VENTING   = 10;
constexpr auto VALVE_CLOSED_THRESHOLD_FUEL_VENTING = 10;

// Washing procedure parameters
constexpr auto VALVE_WIGGLE_DELAY = 1000;

constexpr auto VALVE_CLOSING_DELAY = 500;

}  // namespace Config
}  // namespace Motor
