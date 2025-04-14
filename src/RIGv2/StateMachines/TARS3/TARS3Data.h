/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <cstdint>
#include <iostream>
#include <string>

namespace RIGv2
{

enum class Tars3Action : uint32_t
{
    READY = 0,
    START,
    WAITING,
    FILLING,
    VENTING,
    MANUAL_STOP,
    MANUAL_ACTION_STOP,
};

inline std::string to_string(Tars3Action action)
{
    switch (action)
    {
        case Tars3Action::READY:
            return "READY";
        case Tars3Action::START:
            return "START";
        case Tars3Action::WAITING:
            return "WAITING";
        case Tars3Action::FILLING:
            return "FILLING";
        case Tars3Action::VENTING:
            return "VENTING";
        case Tars3Action::MANUAL_STOP:
            return "MANUAL_STOP";
        case Tars3Action::MANUAL_ACTION_STOP:
            return "MANUAL_ACTION_STOP";
        default:
            return "UNKNOWN";
    }
}

struct Tars3ActionData
{
    uint64_t timestamp = 0;
    Tars3Action action = Tars3Action::READY;

    static std::string header() { return "timestamp,action,actionName\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)action << "," << to_string(action)
           << "\n";
    }
};

struct Tars3SampleData
{
    uint64_t timestamp = 0;
    float pressure     = 0;
    float mass         = 0;

    static std::string header() { return "timestamp,pressure,mass\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << pressure << "," << mass << "\n";
    }
};

}  // namespace RIGv2
