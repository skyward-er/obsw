/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Federico Mandelli, Angelo Prete
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

#include <utils/Constants.h>

#include <ostream>

namespace Parafoil
{

namespace WingConfig
{
// Algorithm configuration

#if defined(CLOSED_LOOP)
constexpr int SELECTED_ALGORITHM = 0;
#elif EARLY_MANEUVER
constexpr int SELECTED_ALGORITHM   = 1;
#elif SEQUENCE
constexpr int SELECTED_ALGORITHM = 2;
#elif ROTATION
constexpr int SELECTED_ALGORITHM = 3;
#else
constexpr int SELECTED_ALGORITHM = 0;
#endif

#if defined(JESOLO)
constexpr float DEFAULT_TARGET_LAT = 45.565835;
constexpr float DEFAULT_TARGET_LON = 12.577307;
#else  // Milan
constexpr float DEFAULT_TARGET_LAT = 45.5013853;
constexpr float DEFAULT_TARGET_LON = 9.1544219;
#endif

constexpr int WING_STRAIGHT_FLIGHT_TIMEOUT = 15 * 1000;  // [ms]

constexpr int WING_UPDATE_PERIOD = 500;  // [ms]

constexpr float PI_CONTROLLER_SATURATION_MAX_LIMIT = Boardcore::Constants::PI;
constexpr float PI_CONTROLLER_SATURATION_MIN_LIMIT = -Boardcore::Constants::PI;

constexpr int GUIDANCE_CONFIDENCE                = 15;
constexpr int GUIDANCE_M1_ALTITUDE_THRESHOLD     = 250;  //[m]
constexpr int GUIDANCE_M2_ALTITUDE_THRESHOLD     = 150;  //[m]
constexpr int GUIDANCE_TARGET_ALTITUDE_THRESHOLD = 50;   //[m]

// TODO check this parameter preflight
constexpr float KP = 1.3537;
constexpr float KI = 0.0111;

constexpr float ALTITUDE_TRIGGER_DEPLOYMENT_ALTITUDE = 300;  // [meters]
constexpr int ALTITUDE_TRIGGER_CONFIDENCE = 10;   // [number of sample]
constexpr int ALTITUDE_TRIGGER_PERIOD     = 100;  //[ms]

constexpr float MAX_SERVO_APERTURE = 0.8;  //[%]

struct WingConfigStruct
{
    float lat                       = DEFAULT_TARGET_LAT;
    float lon                       = DEFAULT_TARGET_LON;
    float maxServoAperture          = MAX_SERVO_APERTURE;
    int wingUpdatePeriod            = WING_UPDATE_PERIOD;
    int altitudeTriggerUpdatePeriod = ALTITUDE_TRIGGER_PERIOD;
    int altitudeTriggerConfidence   = ALTITUDE_TRIGGER_CONFIDENCE;
    int altitudeTriggerDeploymentAltitude =
        ALTITUDE_TRIGGER_DEPLOYMENT_ALTITUDE;
    int selectedAlgorithm = SELECTED_ALGORITHM;

    static std::string header()
    {
        return "DEFAULT_TARGET_LAT,DEFAULT_TARGET_LON,MAX_SERVO_APERTURE,WING_"
               "UPDATE_PERIOD,ALTITUDE_TRIGGER_PERIOD,ALTITUDE_TRIGGER_"
               "CONFIDENCE,ALTITUDE_TRIGGER_DEPLOYMENT_ALTITUDE,SELECTED_"
               "ALGORITHM\n";
    }

    void print(std::ostream& os) const
    {
        os << lat << "," << lon << "," << maxServoAperture << ","
           << wingUpdatePeriod << "," << altitudeTriggerUpdatePeriod << ","
           << altitudeTriggerConfidence << ","
           << altitudeTriggerDeploymentAltitude << "," << selectedAlgorithm
           << "\n";
    }
};

}  // namespace WingConfig

}  // namespace Parafoil
