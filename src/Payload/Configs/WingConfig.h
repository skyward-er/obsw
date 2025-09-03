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

#include <array>
#include <chrono>

namespace Payload
{
namespace Config
{
namespace Wing
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto UPDATE_RATE                  = 1_hz;
constexpr auto TARGET_UPDATE_RATE           = 10_hz;
constexpr auto STRAIGHT_FLIGHT_TIMEOUT      = 15s;
constexpr auto PROGRESSIVE_ROTATION_TIMEOUT = 5s;
constexpr auto COMMAND_PERIOD               = 6s;
constexpr auto WING_DECREMENT               = 20.0f;  // [deg]

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
#if defined(EUROC)
constexpr auto TARGET_LAT = 39.38479088598281f;
constexpr auto TARGET_LON = -8.28564625292085f;
#elif defined(ROCCARASO)
constexpr auto TARGET_LAT = 41.805101925447715f;
constexpr auto TARGET_LON = 14.053726810082074f;
#else  // Milan
constexpr auto TARGET_LAT = 45.5014089f;
constexpr auto TARGET_LON = 9.1543615f;
#endif

constexpr auto ALGORITHM = AlgorithmId::CLOSED_LOOP;
}  // namespace Default

/**
 * @brief Dynamic target configuration. If enabled, the target is not fixed to
 * the one specified in the Default config, but is dynamically set to a fixed
 * offset relative to the position of launch (pin detach).
 */
namespace DynamicTarget
{

constexpr auto ENABLED          = false;
constexpr auto LATITUDE_OFFSET  = 0;  // [m]
constexpr auto LONGITUDE_OFFSET = 0;  // [m]

}  // namespace DynamicTarget

constexpr auto CUTTERS_TIMEOUT = 1s;

namespace PI
{
constexpr auto SATURATION_MIN_LIMIT = -Boardcore::Constants::PI * 0.65;
constexpr auto SATURATION_MAX_LIMIT = Boardcore::Constants::PI * 0.65;

constexpr auto KP = 0.9f;
constexpr auto KI = 0.05f;
}  // namespace PI

namespace Guidance
{
constexpr auto CONFIDENCE                = 15;   // [samples]
constexpr auto M1_ALTITUDE_THRESHOLD     = 250;  // [m]
constexpr auto M2_ALTITUDE_THRESHOLD     = 150;  // [m]
constexpr auto TARGET_ALTITUDE_THRESHOLD = 50;   // [m]
}  // namespace Guidance

namespace Deployment
{

constexpr auto PUMP_DELAY = 2s;

struct Pump
{
    std::chrono::milliseconds flareTime;
    std::chrono::milliseconds resetTime;
};

constexpr std::array<Pump, 3> PUMPS = {
    Pump{.flareTime = 2s, .resetTime = 1s},
    Pump{.flareTime = 2s, .resetTime = 1s},
    Pump{.flareTime = 1s, .resetTime = 500ms},
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

constexpr auto DEPLOYMENT_ALTITUDE = 600;  // [meters]
constexpr auto CONFIDENCE          = 10;   // [samples]
constexpr auto UPDATE_RATE         = 10_hz;

}  // namespace AltitudeTrigger

}  // namespace Config
}  // namespace Payload
