/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <units/Angle.h>
#include <units/Frequency.h>
#include <utils/Constants.h>

#include <array>
#include <chrono>

namespace Main
{
namespace Config
{
namespace Wing
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Angle;

constexpr auto ALTITUDE_MAP_FILENAME = "/sd/altitude_map.bin";

constexpr auto UPDATE_RATE = 20_hz;

namespace Default
{
#if defined(EUROC)
constexpr auto TARGET_LAT = 39.38724722f;
constexpr auto TARGET_LON = -8.28647778f;
#elif defined(ROCCARASO)
constexpr auto TARGET_LAT = 42.2247092f;
constexpr auto TARGET_LON = 13.4310024f;
#else  // Milan
constexpr auto TARGET_LAT = 45.5014089f;
constexpr auto TARGET_LON = 9.1543615f;
#endif
}  // namespace Default
namespace Deployment
{

constexpr auto PUMP_DELAY       = 4s;
constexpr auto PUMP_TIMEOUT     = 15s;
constexpr auto PUMP_ANGLE_LEFT  = 720_deg;
constexpr auto PUMP_ANGLE_RIGHT = 720_deg;

struct Pump
{
    std::chrono::milliseconds pumpTime;
    std::chrono::milliseconds resetTime;
};

// Pumps are ordered from the first to activate to the last
constexpr std::array<Pump, 2> PUMPS = {
    Pump{.pumpTime = 1s, .resetTime = 500ms},
    Pump{.pumpTime = 2s, .resetTime = 1s},
};

}  // namespace Deployment

namespace LandingFlareConfig
{

constexpr bool ENABLED = true;

constexpr float ALTITUDE         = 20;  // [m]
constexpr int CONFIDENCE         = 10;  // [samples]
constexpr auto DURATION          = 360s;
constexpr auto FLARE_ANGLE_LEFT  = 972_deg;
constexpr auto FLARE_ANGLE_RIGHT = 972_deg;

}  // namespace LandingFlareConfig

}  // namespace Wing
}  // namespace Config
}  // namespace Main
