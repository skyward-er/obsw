/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli
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

namespace Payload
{

namespace WingConfig
{
// Algorithm configuration

#if defined(EUROC)
constexpr float DEFAULT_TARGET_LAT = 39.389733;
constexpr float DEFAULT_TARGET_LON = -8.288992;
#elif defined(ROCCARASO)
constexpr float DEFAULT_TARGET_LAT = 41.809216;
constexpr float DEFAULT_TARGET_LON = 14.055310;
#else  // Milan
constexpr float DEFAULT_TARGET_LAT = 41.809216;
constexpr float DEFAULT_TARGET_LON = 14.055310;
#endif

constexpr int WING_UPDATE_PERIOD = 1000;  // [ms]

constexpr float PI_CONTROLLER_SATURATION_MAX_LIMIT = 2.09439;
constexpr float PI_CONTROLLER_SATURATION_MIN_LIMIT = -2.09439;

constexpr int GUIDANCE_CONFIDENCE                = 15;
constexpr int GUIDANCE_M1_ALTITUDE_THRESHOLD     = 250;  //[m]
constexpr int GUIDANCE_M2_ALTITUDE_THRESHOLD     = 150;  //[m]
constexpr int GUIDANCE_TARGET_ALTITUDE_THRESHOLD = 50;   //[m]

// TODO check this parameter preflight
constexpr float KP                                   = 0.4;   //[m]
constexpr float KI                                   = 0.08;  //[m]
constexpr float ALTITUDE_TRIGGER_DEPLOYMENT_ALTITUDE = 300;   // [meters]

constexpr int ALTITUDE_TRIGGER_CONFIDENCE = 10;   // [number of sample]
constexpr int ALTITUDE_TRIGGER_PERIOD     = 100;  //[ms]
}  // namespace WingConfig

}  // namespace Payload
