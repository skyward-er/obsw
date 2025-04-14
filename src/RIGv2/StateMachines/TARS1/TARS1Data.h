/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

enum class Tars1ActionType : uint8_t
{
    READY = 0,
    WASHING,
    OPEN_FILLING,
    OPEN_VENTING,
    CHECK_PRESSURE,
    CHECK_MASS,
    AUTOMATIC_STOP,
    MANUAL_STOP,
};

struct Tars1ActionData
{
    uint64_t timestamp;
    Tars1ActionType action;

    Tars1ActionData() : timestamp{0}, action{Tars1ActionType::READY} {}

    Tars1ActionData(uint64_t timestamp, Tars1ActionType action)
        : timestamp{timestamp}, action{action}
    {
    }

    static std::string header() { return "timestamp,action\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)action << "\n";
    }
};

struct Tars1SampleData
{
    uint64_t timestamp;
    float pressure;
    float mass;

    Tars1SampleData() : timestamp{0}, pressure{0}, mass{0} {}

    Tars1SampleData(uint64_t timestamp, float pressure, float mass)
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
