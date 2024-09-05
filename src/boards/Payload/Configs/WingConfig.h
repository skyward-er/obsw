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

namespace Payload
{
namespace Config
{
namespace Wing
{

// Algorithm configuration
#if defined(CLOSED_LOOP)
constexpr int SELECTED_ALGORITHM = 0;
#elif EARLY_MANEUVER
constexpr int SELECTED_ALGORITHM   = 1;
#elif SEQUENCE
constexpr int SELECTED_ALGORITHM   = 2;
#elif ROTATION
constexpr int SELECTED_ALGORITHM = 3;
#else
constexpr int SELECTED_ALGORITHM = 0;
#endif

#if defined(EUROC)
constexpr float DEFAULT_TARGET_LAT = 39.389733;
constexpr float DEFAULT_TARGET_LON = -8.288992;
#elif defined(ROCCARASO)
constexpr float DEFAULT_TARGET_LAT = 41.8089005;
constexpr float DEFAULT_TARGET_LON = 14.0546716;
#else  // Milan
constexpr float DEFAULT_TARGET_LAT = 45.5010679;
constexpr float DEFAULT_TARGET_LON = 9.1563769;
#endif

constexpr int WING_STRAIGHT_FLIGHT_TIMEOUT = 15 * 1000;  // [ms]

constexpr int WING_UPDATE_PERIOD = 1000;  // [ms]

constexpr float PI_CONTROLLER_SATURATION_MAX_LIMIT = Boardcore::Constants::PI;
constexpr float PI_CONTROLLER_SATURATION_MIN_LIMIT = -Boardcore::Constants::PI;

constexpr int GUIDANCE_CONFIDENCE                = 15;
constexpr int GUIDANCE_M1_ALTITUDE_THRESHOLD     = 250;  //[m]
constexpr int GUIDANCE_M2_ALTITUDE_THRESHOLD     = 150;  //[m]
constexpr int GUIDANCE_TARGET_ALTITUDE_THRESHOLD = 50;   //[m]

// TODO check this parameter preflight
constexpr float KP = 0.9;
constexpr float KI = 0.05;

constexpr float TWIRL_RADIUS = 0.5;  // [%]

}  // namespace Wing

namespace AltitudeTrigger
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto DEPLOYMENT_ALTITUDE = 300;  // [meters]
constexpr auto CONFIDENCE          = 10;   // [samples]
constexpr auto UPDATE_RATE         = 10_hz;
}  // namespace AltitudeTrigger

}  // namespace Config
}  // namespace Payload
