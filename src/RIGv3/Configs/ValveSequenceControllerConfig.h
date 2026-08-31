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

namespace RIGv3
{
namespace Config
{
enum MotorValveBit : uint8_t
{
    MAIN_OX_VALVE_BIT      = 0,
    MAIN_FUEL_VALVE_BIT    = 1,
    PRZ_OX_VALVE_BIT       = 2,
    PRZ_FUEL_VALVE_BIT     = 3,
    OX_VENTING_VALVE_BIT   = 4,
    FUEL_VENTING_VALVE_BIT = 5
};

enum RIGValveBit : uint8_t
{
    PRZ_3WAY_VALVE_BIT    = 0,
    PRZ_FILLING_VALVE_BIT = 1,
    PRZ_RELEASE_VALVE_BIT = 2,
    OX_FILLING_VALVE_BIT  = 3,
    OX_RELEASE_VALVE_BIT  = 4
};

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr uint8_t VALVE_OPENING_THRESHOLD_MAIN_OX      = 60;  // [percent]
constexpr uint8_t VALVE_OPENING_THRESHOLD_MAIN_FUEL    = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_PRZ_OX       = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_PRZ_FUEL     = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_OX_VENTING   = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_FUEL_VENTING = 60;

constexpr uint8_t VALVE_OPENING_THRESHOLD_PRZ_3WAY    = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_PRZ_FILLING = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_PRZ_RELEASE = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_OX_FILLING  = 60;
constexpr uint8_t VALVE_OPENING_THRESHOLD_OX_RELEASE  = 60;

constexpr uint8_t VALVE_CLOSED_THRESHOLD_MAIN_OX      = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_MAIN_FUEL    = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_PRZ_OX       = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_PRZ_FUEL     = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_OX_VENTING   = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_FUEL_VENTING = 10;

constexpr uint8_t VALVE_CLOSED_THRESHOLD_PRZ_3WAY    = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_PRZ_FILLING = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_PRZ_RELEASE = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_OX_FILLING  = 10;
constexpr uint8_t VALVE_CLOSED_THRESHOLD_OX_RELEASE  = 10;

constexpr auto VALVE_WIGGLE_DELAY  = 1000;  // [ms]
constexpr auto VALVE_CLOSING_DELAY = 500;   // [ms]

}  // namespace Config
}  // namespace RIGv3
