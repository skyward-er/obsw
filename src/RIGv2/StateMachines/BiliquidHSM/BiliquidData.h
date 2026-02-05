/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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
#include <reflect.hpp>
#include <string>

namespace RIGv2
{

enum class BiliquidState : uint8_t
{
    IDLE = 0,
    READY,
    SEQUENCE_1,
    SEQUENCE_2_FUEL,
    SEQUENCE_2_OX,
    SEQUENCE_3,
    INVALID,
};

inline std::string to_string(BiliquidState state)
{
    switch (state)
    {
        case BiliquidState::IDLE:
            return "IDLE";
        case BiliquidState::READY:
            return "READY";
        case BiliquidState::SEQUENCE_1:
            return "SEQUENCE_1";
        case BiliquidState::SEQUENCE_2_FUEL:
            return "SEQUENCE_2_FUEL";
        case BiliquidState::SEQUENCE_2_OX:
            return "SEQUENCE_2_OX";
        case BiliquidState::SEQUENCE_3:
            return "SEQUENCE_3";
        default:
            return "UNKNOWN";
    }
}

struct BiliquidData
{
    uint64_t timestamp;
    BiliquidState state;

    BiliquidData() : timestamp{0}, state{BiliquidState::IDLE} {}

    BiliquidData(uint64_t timestamp, BiliquidState state)
        : timestamp{timestamp}, state{state}
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(BiliquidData, FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

}  // namespace RIGv2
