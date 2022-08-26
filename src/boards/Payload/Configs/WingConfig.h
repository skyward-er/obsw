/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

namespace Payload
{

namespace WingConfig
{

// Algorithm configuration
constexpr uint32_t WING_UPDATE_PERIOD = 100;  // [ms]
constexpr uint8_t WING_CONTROLLER_ID  = 100;  // TODO define a correct ID

// Arm, start and flare thresholds
constexpr float WING_ALGORITHM_ARM_ALTITUDE   = 250;   // [m]
constexpr float WING_ALGORITHM_START_ALTITUDE = 200;   // [m]
constexpr float WING_FLARE_ALTITUDE           = 1275;  // [m]

constexpr float WING_CALIBRATION_PRESSURE    = 101325;  // [Pa]
constexpr float WING_CALIBRATION_TEMPERATURE = 300;     // [K]
constexpr uint8_t WING_PRESSURE_MEAN_COUNT   = 20;

// TODO change the values below
constexpr float DEFAULT_GPS_INITIAL_LAT = 42.5;
constexpr float DEFAULT_GPS_INITIAL_LON = 9.5;

constexpr float DEFAULT_TARGET_LAT = 42;
constexpr float DEFAULT_TARGET_LON = 9;

// Wing altitude checker configs
constexpr float WING_ALTITUDE_CHECKER_TASK_ID = 230;
constexpr float WING_ALTITUDE_CHECKER_PERIOD  = 100;  // [ms]
constexpr float WING_ALTITUDE_REFERENCE       = 400;

}  // namespace WingConfig

}  // namespace Payload
