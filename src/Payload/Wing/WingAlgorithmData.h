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
#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace Payload
{
/**
 * This class represents the algorithm data structure that needs to be logged
 * into the onboard SD card. It has the timestamp(absolute) and the servo
 * position set by the selected algorithm
 */
struct WingAlgorithmData
{
    uint64_t timestamp  = 0;  // First timestamp is 0 (in microseconds)
    float servo1Angle   = 0;  // degrees
    float servo2Angle   = 0;  // degrees
    float targetAngle   = 0;  // radians (automatic only)
    float velocityAngle = 0;  // radians
    float targetX       = 0;  // NED (only automatic algorithm)
    float targetY       = 0;  // NED (only automatic algorithm)
    float error         = 0;
    float pidOutput     = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(WingAlgorithmData,
                          FIELD_DEF(timestamp) FIELD_DEF(servo1Angle)
                              FIELD_DEF(servo2Angle) FIELD_DEF(targetAngle)
                                  FIELD_DEF(velocityAngle) FIELD_DEF(targetX)
                                      FIELD_DEF(targetY) FIELD_DEF(error)
                                          FIELD_DEF(pidOutput));
    }
};
}  // namespace Payload
