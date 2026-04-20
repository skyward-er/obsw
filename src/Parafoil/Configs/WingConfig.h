/* Copyright (c) 2024-2026 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete, Niccolò Betto, Raul Radu
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
#include <utils/Constants.h>

#include <array>
#include <chrono>

#include "MapData.h"

namespace Parafoil
{
namespace Config
{
namespace Wing
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Angle;

constexpr auto UPDATE_RATE             = 10_hz;
constexpr auto TARGET_UPDATE_RATE      = 10_hz;
constexpr auto SERVO_UPDATE_RATE       = 50_hz;
constexpr auto STRAIGHT_FLIGHT_TIMEOUT = 15s;

// progressive rotation
constexpr auto PROGRESSIVE_ROTATION_TIMEOUT = 500ms;
constexpr auto COMMAND_PERIOD               = 6s;
constexpr auto WING_DECREMENT               = 90_deg;
constexpr auto INITIAL_ANGLE                = 720_deg;

constexpr auto SERVO_LEFT_MIN_ANGLE = 0_deg;
constexpr auto SERVO_LEFT_MAX_ANGLE = 1080_deg;

constexpr auto SERVO_RIGHT_MAX_ANGLE = 0_deg;
constexpr auto SERVO_RIGHT_MIN_ANGLE = -1080_deg;

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
    FROM_FILE,
    LAST  ///< Used to count the number of algorithms
};

namespace Default
{
#if defined(JESOLO)
constexpr auto TARGET_LAT = 45.565652923793806f;
constexpr auto TARGET_LON = 12.57763990872353f;
#else  // Milan
// B12
constexpr auto TARGET_LAT = 45.5013853;
constexpr auto TARGET_LON = 9.1544219;

// Via Negri
// constexpr auto TARGET_LAT = 45.50058226119641;
// constexpr auto TARGET_LON = 9.157215417984244;
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
#elif defined(ALGORITHM_FILE_SEQUENCE)
constexpr auto ALGORITHM = AlgorithmId::FROM_FILE;
#else
constexpr auto ALGORITHM = AlgorithmId::CLOSED_LOOP;
#endif
}  // namespace Default

constexpr std::initializer_list<float> PROGRESSIVE_ASYMMETRIC_SEQUENCE = {
    0.8f, 0.6f, 0.4f};

/**
 * @brief Dynamic target configuration. If enabled, the target is not fixed to
 * the one specified in the Default config, but is dynamically set to a fixed
 * offset relative to the position of launch (pin detach).
 */
namespace DynamicTarget
{

#if defined(DYNAMIC_TARGET)
constexpr auto ENABLED = true;
#else
constexpr auto ENABLED = false;
#endif

#if defined(DYNAMIC_TARGET_LATITUDE_OFFSET)
constexpr auto LATITUDE_OFFSET = DYNAMIC_TARGET_LATITUDE_OFFSET;
#else
constexpr auto LATITUDE_OFFSET = 0;
#endif

#if defined(DYNAMIC_TARGET_LONGITUDE_OFFSET)
constexpr auto LONGITUDE_OFFSET = DYNAMIC_TARGET_LONGITUDE_OFFSET;
#else
constexpr auto LONGITUDE_OFFSET = 0;
#endif

}  // namespace DynamicTarget

constexpr auto CUTTERS_TIMEOUT = 1s;

namespace PI
{
// constexpr auto SATURATION_MIN_LIMIT = -Boardcore::Constants::PI * 0.65;
// constexpr auto SATURATION_MAX_LIMIT = Boardcore::Constants::PI * 0.65;
constexpr auto GAIN                 = 4.0f;
constexpr auto SATURATION_MIN_LIMIT = -Boardcore::Constants::PI * 4;
constexpr auto SATURATION_MAX_LIMIT = Boardcore::Constants::PI * 4;
constexpr auto KP                   = 0.9f * GAIN;
constexpr auto KI                   = 0.0f;
// constexpr auto KI                   = 0.05f * GAIN;
}  // namespace PI

namespace Guidance
{
/* linter off */ using namespace Boardcore::Units::Length;

constexpr auto CONFIDENCE                = 15;     // [samples]
constexpr auto M1_ALTITUDE_THRESHOLD     = 250_m;  // [m]
constexpr auto M2_ALTITUDE_THRESHOLD     = 150_m;  // [m]
constexpr auto TARGET_ALTITUDE_THRESHOLD = 50_m;   // [m]
}  // namespace Guidance

// Early Maneuver Guidance EMC point generation parameters
constexpr float LATERAL_DISTANCE = 30.0;
constexpr float SCALE_FACTOR     = 1.1;

namespace Deployment
{

constexpr auto PUMP_DELAY       = 2s;
constexpr auto PUMP_ANGLE_LEFT  = 720_deg;
constexpr auto PUMP_ANGLE_RIGHT = -720_deg;
struct Pump
{
    std::chrono::milliseconds flareTime;
    std::chrono::milliseconds resetTime;
};

// Pumps are ordered from the first to activate to the last
constexpr std::array<Pump, 2> PUMPS = {
    Pump{.flareTime = 100ms, .resetTime = 500ms},
    Pump{.flareTime = 100ms, .resetTime = 500ms},
};

}  // namespace Deployment

namespace LandingFlare
{

/* linter off */ using namespace Boardcore::Units::Length;

#ifdef DISABLE_LANDING_FLARE
constexpr auto ENABLED = false;
#else
constexpr auto ENABLED = true;
#endif

#if defined(JESOLO)
constexpr Meter ALTITUDE = 30_m;  // [m]
#elif defined(MILANO)
constexpr Meter ALTITUDE = 5.0_m;  // [m]
#else
constexpr Meter ALTITUDE = 30_m;  // [m]
#endif
constexpr int CONFIDENCE         = 10;  // [samples]
constexpr auto UPDATE_RATE       = 50_hz;
constexpr auto DURATION          = 30s;
constexpr auto ANGLE_LEFT_SERVO  = 1080_deg;
constexpr auto ANGLE_RIGHT_SERVO = -1080_deg;

namespace TinyPull
{

#if defined(TINY_PULL)
#if !defined(MILANO) && !defined(JESOLO)
#error \
    "Tiny Pulls should be used only for testing and can be enabled only in MILANO and JESOLO"
#endif

constexpr auto ENABLED = true;
#else
constexpr auto ENABLED = false;
#endif
constexpr auto ANGLE_LEFT_SERVO  = 90_deg;
constexpr auto ANGLE_RIGHT_SERVO = -90_deg;
#if defined(JESOLO)
constexpr std::initializer_list<Meter> ALTITUDE_THRESHOLDS = {30_m, 25_m, 20_m,
                                                              15_m, 10_m, 5_m};
#elif defined(MILANO)
constexpr std::initializer_list<Meter> ALTITUDE_THRESHOLDS = {5_m, 4_m, 3_m};
#else
constexpr std::initializer_list<Meter> ALTITUDE_THRESHOLDS = {30_m, 25_m, 20_m,
                                                              15_m, 10_m, 5_m};
#endif
}  // namespace TinyPull

constexpr auto ALTITUDE_MAP_ADDRESS = map_data_bin;

}  // namespace LandingFlare

namespace Rotation
{
constexpr auto ROTATION_PERIOD = 8s;  ///< Period of the rotation maneuver
constexpr std::initializer_list<Degree> ROTATION_LEFT  = {720_deg, 360_deg,
                                                          180_deg, 0_deg};
constexpr std::initializer_list<Degree> ROTATION_RIGHT = {-720_deg, -360_deg,
                                                          -180_deg, 0_deg};
}  // namespace Rotation

}  // namespace Wing

namespace AltitudeTrigger
{

/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Length;

constexpr Meter DEPLOYMENT_ALTITUDE = 450_m;  // [meters]
constexpr auto CONFIDENCE           = 10;     // [samples]
constexpr auto UPDATE_RATE          = 10_hz;

}  // namespace AltitudeTrigger

}  // namespace Config
}  // namespace Parafoil
