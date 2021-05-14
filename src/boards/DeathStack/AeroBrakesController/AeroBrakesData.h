/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <stdint.h>

#include <iostream>
#include <string>

namespace DeathStackBoard
{

/**
 * Enum defining the possibile FSM states.
 */
enum AeroBrakesControllerState : uint8_t
{
    IDLE = 0,
    SHADOW_MODE,
    ENABLED,
    END,
    DISABLED,
    TEST_AEROBRAKES,
};

/**
 * Structure defining the overall controller status.
 */
struct AeroBrakesControllerStatus
{
    long long timestamp;
    AeroBrakesControllerState state;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

/**
 * Structure defining the output of the control algorithm.
 */
struct AeroBrakesData
{
    long long timestamp;
    bool running;
    float servo_position;

    static std::string header() { return "timestamp,running,servo_position\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)running << "," << servo_position << "\n";
    }
};

}  // namespace DeathStackBoard
