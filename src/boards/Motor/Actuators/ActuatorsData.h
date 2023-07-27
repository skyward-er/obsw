/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Alberto Nidasio
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

#include <ostream>

namespace Motor
{

struct ActuatorsData
{
    uint64_t timestamp;
    uint8_t servoId;
    float position;

    ActuatorsData()
    {
        timestamp = 0;
        servoId   = 0;
        position  = 0;
    }

    ActuatorsData(uint64_t time, uint8_t servo, float pos)
        : timestamp(time), servoId(servo), position(pos)
    {
    }

    static std::string header() { return "timestamp,servoId,position\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)servoId << "," << position << "\n";
    }
};

}  // namespace Motor