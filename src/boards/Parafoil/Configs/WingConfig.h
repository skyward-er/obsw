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
// TODO put things to change between flight in this file
namespace Parafoil
{

namespace WingConfig
{
// Algorithm configuration
constexpr int WING_UPDATE_PERIOD           = 100;  // [ms]
constexpr int WING_ALTITUDE_TRIGGER_PERIOD = 100;  //[ms]

#if defined(EUROC)
constexpr float DEFAULT_TARGET_LAT = 39.389733;
constexpr float DEFAULT_TARGET_LON = -8.288992;
#elif defined(ROCCARASO)
constexpr float DEFAULT_TARGET_LAT = 41.8039952;
constexpr float DEFAULT_TARGET_LON = 14.0547223;
#elif defined(TERNI)
constexpr float DEFAULT_TARGET_LAT = 42.572165;
constexpr float DEFAULT_TARGET_LON = 12.585847;
#else  // Milan
constexpr float DEFAULT_TARGET_LAT = 42.572165;
constexpr float DEFAULT_TARGET_LON = 12.585847;
/*constexpr float DEFAULT_TARGET_LAT = 42;
constexpr float DEFAULT_TARGET_LON = 9;*/ //to be safe
#endif

// Wing altitude checker configs
constexpr int WING_ALTITUDE_TRIGGER_CONFIDENCE = 5;   // [number of sample]
constexpr int WING_ALTITUDE_TRIGGER_FALL       = 50;  // [meters]
constexpr int WING_STRAIGHT_FLIGHT_TIMEOUT     = 15 * 1000000;  // [us]

}  // namespace WingConfig

}  // namespace Parafoil
