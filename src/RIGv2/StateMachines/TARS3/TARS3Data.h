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
    WAITING_CYCLE,
    WAITING_FILLING,
    PRESSURE_STABLE_CYCLE,
    PRESSURE_STABLE_FILLING,
    FILLING,
    VENTING,
    MANUAL_STOP,
    MANUAL_ACTION_STOP,
};

static std::array<const char*, 10> TARS3_ACTION_STRINGS = {
    "READY",
    "START",
    "WAITING_CYCLE",
    "WAITING_FILLING",
    "PRESSURE_STABLE_CYCLE",
    "PRESSURE_STABLE_FILLING",
    "FILLING",
    "VENTING",
    "MANUAL_STOP",
    "MANUAL_ACTION_STOP",
};

static std::array<const char*, 10> TARS3_ACTION_DATA_TYPE = {
    "",                // READY
    "",                // START
    "PRESSURE_DELTA",  // WAITING_CYCLE
    "PRESSURE_DELTA",  // WAITING_FILLING
    "PRESSURE_DELTA",  // PRESSURE_STABLE_CYCLE
    "PRESSURE_DELTA",  // PRESSURE_STABLE_FILLING
    "OPEN_TIME",       // FILLING
    "OPEN_TIME",       // VENTING
    "",                // MANUAL_STOP
    "",                // MANUAL_ACTION_STOP
};

inline std::ostream& operator<<(std::ostream& os, Tars3Action action)
{
    os << TARS3_ACTION_STRINGS[static_cast<uint32_t>(action)];
    return os;
}

struct Tars3ActionData
{
    uint64_t timestamp = 0;
    Tars3Action action = Tars3Action::READY;
    float data         = 0;  // Additional data attached to the action

    static std::string header()
    {
        return "timestamp,action,actionName,data,dataType\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)action << "," << action << "," << data
           << "," << TARS3_ACTION_DATA_TYPE[static_cast<uint32_t>(action)]
           << "\n";
    }
};

struct Tars3SampleData
{
    uint64_t timestamp;
    float pressure;
    float mass;

    Tars3SampleData() : timestamp{0}, pressure{0}, mass{0} {}

    Tars3SampleData(uint64_t timestamp, float pressure, float mass)
        : timestamp{timestamp}, pressure{pressure}, mass{mass}
    {
    }

    static std::string header() { return "timestamp,pressure,mass\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << pressure << "," << mass << "\n";
    }
};

}  // namespace RIGv2
