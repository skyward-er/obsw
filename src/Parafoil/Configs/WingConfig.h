/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto, Davide Basso
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
#include <units/Length.h>
#include <units/Time.h>
#include <utils/Constants.h>

namespace Parafoil
{
namespace Config
{
namespace Wing
{

/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Length;
/* linter off */ using namespace Boardcore::Units::Time;
/* linter off */ using namespace Boardcore::Units::Angle;

constexpr auto UPDATE_RATE                  = 1_hz;
constexpr auto TARGET_UPDATE_RATE           = 10_hz;
constexpr auto STRAIGHT_FLIGHT_TIMEOUT      = 15_s;
constexpr auto PROGRESSIVE_ROTATION_TIMEOUT = 5_s;
constexpr auto COMMAND_PERIOD               = 6_s;
constexpr auto WING_DECREMENT               = 20_deg;

/**
 * @brief The available algorithms for the wing controller.
 */
enum class AlgorithmId : size_t
{
    CLOSED_LOOP = 0,
    EARLY_MANEUVER,
    SEQUENCE,
    ROTATION,
    PROGRESSIVE_ROTATION,
    LAST  ///< Used to count the number of algorithms
};

namespace Default
{
#if defined(JESOLO)
constexpr auto TARGET_LAT = 45.565264f;
constexpr auto TARGET_LON = 12.577050f;
#else  // Milan
constexpr auto TARGET_LAT = 45.5013853;
constexpr auto TARGET_LON = 9.1544219;
#endif

#if defined(ALGORITHM_CLOSED_LOOP)
constexpr auto ALGORITHM = AlgorithmId::CLOSED_LOOP;
#elif defined(ALGORITHM_EARLY_MANEUVER)
constexpr auto ALGORITHM = AlgorithmId::EARLY_MANEUVER;
#elif defined(ALGORITHM_SEQUENCE)
constexpr auto ALGORITHM = AlgorithmId::SEQUENCE;
#elif defined(ALGORITHM_ROTATION)
constexpr auto ALGORITHM = AlgorithmId::ROTATION;
#elif defined(ALGORITHM_PROGRESSIVE_ROTATION)
constexpr auto ALGORITHM = AlgorithmId::PROGRESSIVE_ROTATION;
#else
constexpr auto ALGORITHM = AlgorithmId::CLOSED_LOOP;
#endif

}  // namespace Default

/**
 * @brief Dynamic target configuration. If enabled, the target is not fixed to
 * the one specified in the Default config, but is dynamically set to a fixed
 * offset relative to the position of launch (pin detach).
 */
namespace DynamicTarget
{

/* linter off */ using namespace Boardcore::Units::Length;

#if defined(DYNAMIC_TARGET)
constexpr auto ENABLED = true;
#else
constexpr auto ENABLED = false;
#endif

#if defined(DYNAMIC_TARGET_LATITUDE_OFFSET)
constexpr auto LATITUDE_OFFSET = Meter{DYNAMIC_TARGET_LATITUDE_OFFSET};
#else
constexpr auto LATITUDE_OFFSET = 0_m;
#endif

#if defined(DYNAMIC_TARGET_LONGITUDE_OFFSET)
constexpr auto LONGITUDE_OFFSET = Meter{DYNAMIC_TARGET_LONGITUDE_OFFSET};
#else
constexpr auto LONGITUDE_OFFSET = 0_m;
#endif

}  // namespace DynamicTarget

namespace PI
{
constexpr auto SATURATION_MIN_LIMIT = -Boardcore::Constants::PI;
constexpr auto SATURATION_MAX_LIMIT = Boardcore::Constants::PI;

constexpr auto KP = 0.9f;
constexpr auto KI = 0.05f;
}  // namespace PI

namespace Guidance
{

constexpr auto CONFIDENCE                = 15;  // [samples]
constexpr auto M1_ALTITUDE_THRESHOLD     = 250_m;
constexpr auto M2_ALTITUDE_THRESHOLD     = 150_m;
constexpr auto TARGET_ALTITUDE_THRESHOLD = 50_m;

}  // namespace Guidance

namespace LandingFlare
{
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Length;

#ifdef DISABLE_LANDING_FLARE
constexpr auto ENABLED = false;
#else
constexpr auto ENABLED = true;
#endif

constexpr auto ALTITUDE    = 15_m;
constexpr auto CONFIDENCE  = 10;  // [samples]
constexpr auto UPDATE_RATE = 10_hz;
constexpr auto DURATION    = 5_s;
}  // namespace LandingFlare

}  // namespace Wing

}  // namespace Config
}  // namespace Parafoil
