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

constexpr auto UPDATE_RATE        = 1_hz;
constexpr auto TARGET_UPDATE_RATE = 10_hz;

namespace Default
{
#if defined(EUROC)
constexpr auto TARGET_LAT = 39.38724722f;
constexpr auto TARGET_LON = -8.28647778f;
#elif defined(ROCCARASO)
constexpr auto TARGET_LAT = 41.807905240570980f;
constexpr auto TARGET_LON = 14.057047761535994f;
#else  // Milan
constexpr auto TARGET_LAT = 45.5014089f;
constexpr auto TARGET_LON = 9.1543615f;
#endif

namespace Deployment
{

constexpr auto PUMP_DELAY = 5s;

struct Pump
{
    std::chrono::milliseconds flareTime;
    std::chrono::milliseconds resetTime;
};

// Pumps are ordered from the first to activate to the last
constexpr std::array<Pump, 3> PUMPS = {
    Pump{.flareTime = 1s, .resetTime = 500ms},
    Pump{.flareTime = 2s, .resetTime = 1s},
    Pump{.flareTime = 2s, .resetTime = 1s},
};

}  // namespace Deployment

namespace LandingFlare
{

constexpr bool ENABLED = false;

constexpr float ALTITUDE   = 15;  // [m]
constexpr int CONFIDENCE   = 10;  // [samples]
constexpr auto UPDATE_RATE = 10_hz;
constexpr auto DURATION    = 5s;

}  // namespace LandingFlare

constexpr auto ROTATION_PERIOD = 10s;  ///< Period of the rotation maneuver

}  // namespace Wing

namespace AltitudeTrigger
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto DEPLOYMENT_ALTITUDE = 450;  // [meters]
constexpr auto CONFIDENCE          = 10;   // [samples]
constexpr auto UPDATE_RATE         = 10_hz;

}  // namespace AltitudeTrigger

}  // namespace Config
}  // namespace Main
