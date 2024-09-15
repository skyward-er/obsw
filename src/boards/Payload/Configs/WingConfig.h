/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto
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

#include <chrono>

namespace Payload
{
namespace Config
{
namespace Wing
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;

/**
 * @brief The available algorithms for the wing controller.
 */
enum class AlgorithmId : size_t
{
    EARLY_MANEUVER = 0,
    CLOSED_LOOP,
    ROTATION,  ///< A predefined sequence of rotation maneuvers
    LAST,      ///< Used to count the number of algorithms
};

namespace Default
{
#if defined(EUROC)
constexpr auto TARGET_LAT = 39.389733f;
constexpr auto TARGET_LON = -8.288992f;
#elif defined(ROCCARASO)
constexpr auto TARGET_LAT = 41.805101925447715f;
constexpr auto TARGET_LON = 14.053726810082074f;
#else  // Milan
constexpr auto TARGET_LAT = 45.5014089f;
constexpr auto TARGET_LON = 9.1543615f;
#endif

constexpr auto ALGORITHM = AlgorithmId::EARLY_MANEUVER;
}  // namespace Default

constexpr auto UPDATE_RATE = 1_hz;

constexpr auto CUTTERS_TIMEOUT = 1s;

constexpr auto FLARE_WAIT     = 5s;  ///< Time to wait before the first flare
constexpr auto FLARE_COUNT    = 2;   ///< Number of flares
constexpr auto FLARE_DURATION = 2s;  ///< Duration of the flare maneuver
constexpr auto FLARE_INTERVAL = 1s;  ///< Interval between two flares

namespace PI
{
constexpr auto SATURATION_MIN_LIMIT = -Boardcore::Constants::PI;
constexpr auto SATURATION_MAX_LIMIT = Boardcore::Constants::PI;

constexpr auto KP = 0.9f;
constexpr auto KI = 0.05f;
}  // namespace PI

namespace Guidance
{
constexpr auto CONFIDENCE                = 10;   // [samples]
constexpr auto M1_ALTITUDE_THRESHOLD     = 250;  // [m]
constexpr auto M2_ALTITUDE_THRESHOLD     = 150;  // [m]
constexpr auto TARGET_ALTITUDE_THRESHOLD = 50;   // [m]
}  // namespace Guidance

constexpr auto ROTATION_PERIOD = 10s;  ///< Period of the rotation maneuver

}  // namespace Wing

namespace AltitudeTrigger
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto DEPLOYMENT_ALTITUDE = 470;  // [meters]
constexpr auto CONFIDENCE          = 10;   // [samples]
constexpr auto UPDATE_RATE         = 10_hz;
}  // namespace AltitudeTrigger

}  // namespace Config
}  // namespace Payload
