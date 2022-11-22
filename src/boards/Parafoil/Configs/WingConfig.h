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

#include <drivers/timer/PWM.h>
#include <miosix.h>

#include <Eigen/Core>

namespace Parafoil
{

namespace WingConfig
{

// Algorithm configuration
constexpr uint32_t WING_UPDATE_PERIOD = 100;  // [ms]
constexpr uint8_t WING_CONTROLLER_ID  = 100;
constexpr uint32_t WIND_PREDICTION_CALIBRATION_TIMEOUT =
    10 * 1000;  // time needed for the first loop [ms]
constexpr uint32_t WIND_PREDICTION_TIMEOUT =
    2 * WIND_PREDICTION_CALIBRATION_TIMEOUT;

constexpr int WIND_CALIBRATION_SAMPLE_NUMBER = 20;

constexpr uint32_t WIND_CALIBRATION_UPDATE_PERIOD =
    (WIND_PREDICTION_CALIBRATION_TIMEOUT) / WIND_CALIBRATION_SAMPLE_NUMBER -
    10;  //[ms]
constexpr uint8_t WIND_CALIBRATION_ID            = 101;
constexpr uint32_t WIND_PREDICTION_UPDATE_PERIOD = 100;  //[ms]
constexpr uint8_t WING_PREDICTION_ID             = 102;

// Arm, start and flare thresholds
constexpr float WING_ALGORITHM_ARM_ALTITUDE   = 250;  // [m]
constexpr float WING_ALGORITHM_START_ALTITUDE = 200;  // [m]
constexpr float WING_FLARE_ALTITUDE           = 10;   // [m]

constexpr float WING_CALIBRATION_PRESSURE    = 101325;  // [Pa]
constexpr float WING_CALIBRATION_TEMPERATURE = 300;     // [K]
constexpr uint8_t WING_PRESSURE_MEAN_COUNT   = 20;

#if defined(EUROC)
constexpr float DEFAULT_TARGET_LAT = 39.389733;
constexpr float DEFAULT_TARGET_LON = -8.288992;
#elif defined(ROCCARASO)
constexpr float DEFAULT_TARGET_LAT = 41.8039952;
constexpr float DEFAULT_TARGET_LON = 14.0547223;
#else  // Milan
constexpr float DEFAULT_TARGET_LAT = 42;
constexpr float DEFAULT_TARGET_LON = 9;
#endif

// Wing altitude checker configs
constexpr float WING_ALTITUDE_CHECKER_TASK_ID  = 230;
constexpr float WING_ALTITUDE_CHECKER_PERIOD   = 100;  // [ms]
constexpr float WING_ALTITUDE_REFERENCE        = 450;
constexpr int WING_ALTITUDE_TRIGGER_CONFIDENCE = 5;
constexpr int WING_ALTITUDE_DESCEND_CONTROL    = 50;  // meter

}  // namespace WingConfig

}  // namespace Parafoil
