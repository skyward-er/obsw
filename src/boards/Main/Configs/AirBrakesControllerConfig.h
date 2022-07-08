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

#include <algorithms/AirBrakes/AirBrakesConfig.h>

namespace Main
{

namespace AirBrakesControllerConfigs
{

static constexpr int SHADOW_MODE_TIMEOUT = 3.5 * 1000;

// Vertical speed limit beyond which the airbrakes need to be disabled.
static constexpr float DISABLE_VERTICAL_SPEED_TARGET = 10.0;

static const Boardcore::AirBrakesConfig ABK_CONFIG{
    .N000       = 0,
    .N100       = 0,
    .N200       = 0,
    .N300       = 0,
    .N400       = 0,
    .N500       = 0,
    .N600       = 0,
    .N010       = 0,
    .N020       = 0,
    .N110       = 0,
    .N120       = 0,
    .N210       = 0,
    .N220       = 0,
    .N310       = 0,
    .N320       = 0,
    .N410       = 0,
    .N420       = 0,
    .N510       = 0,
    .N520       = 0,
    .N001       = 0,
    .EXTENSION  = 0,
    .DRAG_STEPS = 0,
    .EXT_POL_1  = 0,
    .EXT_POL_2  = 0,
    .EXT_POL_3  = 0,
    .EXT_POL_4  = 0,
    .S0         = 0,
    .SURFACE    = 0,
    .KP         = 0,
    .KI         = 0,
    .TS         = 0,
};

}  // namespace AirBrakesControllerConfigs

}  // namespace Main
