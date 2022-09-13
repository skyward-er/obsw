/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/Configs/ActuatorsConfigs.h>
#include <algorithms/AirBrakes/AirBrakesConfig.h>

namespace Main
{

namespace AirBrakesControllerConfig
{

constexpr uint32_t UPDATE_PERIOD = 100;  // 10 hz

#ifdef EUROC
static constexpr int SHADOW_MODE_TIMEOUT = 5 * 1000;
#else
static constexpr int SHADOW_MODE_TIMEOUT = 3.5 * 1000;
#endif

constexpr float MACH_LIMIT = 0.8;

// Vertical speed limit beyond which the airbrakes need to be disabled.
constexpr float DISABLE_VERTICAL_SPEED_TARGET = 10.0;

static const Boardcore::AirBrakesConfig BASE_ABK_CONFIG{
    .N000       = 0.4884,
    .N100       = -1.4391,
    .N200       = 6.6940,
    .N300       = -18.4272,
    .N400       = 29.1044,
    .N500       = -24.5585,
    .N600       = 8.6058,
    .N010       = 9.0426,
    .N020       = 159.5995,
    .N110       = 4.8188,
    .N120       = -208.4471,
    .N210       = 47.0771,
    .N220       = 1.9433e+03,
    .N310       = -205.6689,
    .N320       = -6.4634e+03,
    .N410       = 331.0332,
    .N420       = 8.8763e+03,
    .N510       = -161.8111,
    .N520       = -3.9917e+03,
    .N001       = 2.8025e-06,
    .EXTENSION  = 0.0373,
    .DRAG_STEPS = 20,
    .EXT_POL_1  = -0.009216,
    .EXT_POL_2  = 0.02492,
    .EXT_POL_3  = -0.01627,
    .EXT_POL_4  = 0.03191,
    .S0         = 0.017671458676443,
    .SURFACE    = 0.009564 * Main::ActuatorsConfigs::ABK_SERVO_ROTATION *
               EIGEN_PI / 180.0,
};

}  // namespace AirBrakesControllerConfig

}  // namespace Main
