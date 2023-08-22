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

#include <stdint.h>

#include <iostream>
#include <string>
namespace Parafoil
{

namespace WingConfig
{
// Algorithm configuration

constexpr int WING_UPDATE_PERIOD           = 100;   // [ms]
constexpr int WING_ALTITUDE_TRIGGER_PERIOD = 1000;  //[ms]

#if defined(EUROC)
constexpr float DEFAULT_TARGET_LAT = 39.389733;
constexpr float DEFAULT_TARGET_LON = -8.288992;
#elif defined(ROCCARASO)
constexpr float DEFAULT_TARGET_LAT = 41.8039952;
constexpr float DEFAULT_TARGET_LON = 14.0547223;
#elif defined(TERNI)
constexpr float DEFAULT_TARGET_LAT = 42.572165;
constexpr float DEFAULT_TARGET_LON = 12.585847;
#elif defined(MOLINELLA)
constexpr float DEFAULT_TARGET_LAT = 44.588923;
constexpr float DEFAULT_TARGET_LON = 11.653212;
#else  // Milan
constexpr float DEFAULT_TARGET_LAT = 45.501148;
constexpr float DEFAULT_TARGET_LON = 9.156301;
#endif

#if defined(GUIDED)

constexpr int SELECTED_ALGORITHM = 0;
#elif STOP_AND_GO
constexpr int SELECTED_ALGORITHM   = 1;
#elif ROTATION
constexpr int SELECTED_ALGORITHM   = 2;
#elif EARLY_MANEUVER
constexpr int SELECTED_ALGORITHM   = 3;
#else
constexpr int SELECTED_ALGORITHM   = 0;
#endif
constexpr float MAX_SERVO_APERTURE = 1.0f;
// Wing altitude checker configs
constexpr int WING_ALTITUDE_TRIGGER_CONFIDENCE = 10;  // [number of sample]
constexpr int WING_ALTITUDE_TRIGGER_FALL       = 50;  // [meters]
constexpr int WING_STRAIGHT_FLIGHT_TIMEOUT     = 15 * 1000000;  // [us]

constexpr float PI_CONTROLLER_SATURATION_MAX_LIMIT = 0.1;
constexpr float PI_CONTROLLER_SATURATION_MIN_LIMIT = -0.1;

constexpr int GUIDANCE_CONFIDENCE                = 15;
constexpr int GUIDANCE_M1_ALTITUDE_THRESHOLD     = 250;  //[m]
constexpr int GUIDANCE_M2_ALTITUDE_THRESHOLD     = 150;  //[m]
constexpr int GUIDANCE_TARGET_ALTITUDE_THRESHOLD = 50;   //[m]

constexpr float KP = 0.1;    //[m]
constexpr float KI = 0.001;  //[m]

struct WingConfigStruct
{

    int updatePeriod    = WING_UPDATE_PERIOD;            // [ms]
    int altitudeTrigger = WING_ALTITUDE_TRIGGER_PERIOD;  //[ms]

    float lat = DEFAULT_TARGET_LAT;
    float lon = DEFAULT_TARGET_LON;

    int selectedAlgo = SELECTED_ALGORITHM;
    float maxServo   = MAX_SERVO_APERTURE;
    // Wing altitude checker configs
    int confidence    = WING_ALTITUDE_TRIGGER_CONFIDENCE;
    int triggerFall   = WING_ALTITUDE_TRIGGER_FALL;
    int flightTimeout = WING_STRAIGHT_FLIGHT_TIMEOUT;
    static std::string header()
    {
        return "WING_UPDATE_PERIOD,WING_ALTITUDE_TRIGGER_PERIOD, "
               "DEFAULT_TARGET_LAT,DEFAULT_TARGET_LON, MAX_SERVO_APERTURE,"
               "WING_ALTITUDE_TRIGGER_CONFIDENCE,"
               "WING_ALTITUDE_TRIGGER_FALL,WING_STRAIGHT_FLIGHT_TIMEOUT,"
               "SELECTED_ALGORITHM \n ";
    }

    void print(std::ostream& os) const
    {
        os << updatePeriod << "," << altitudeTrigger << "," << lat << "," << lon
           << "," << maxServo << "," << confidence << "," << triggerFall << ","
           << flightTimeout << "," << selectedAlgo << "\n";
    }
};

}  // namespace WingConfig

}  // namespace Parafoil
