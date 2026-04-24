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

namespace Motor
{

enum class FiringSequenceState : uint8_t
{
    IDLE = 0,
    READY,
    IGNITER,
    IGNITER_WAIT,
    PILOT_FLAME,
    PILOT_FLAME_WAIT,
    RAMP_UP,
    FULL_THROTTLE,
    LOW_THROTTLE,
    ENDED,
    INVALID,
};

inline std::string to_string(FiringSequenceState state)
{
    switch (state)
    {
        case FiringSequenceState::IDLE:
            return "IDLE";
        case FiringSequenceState::READY:
            return "READY";
        case FiringSequenceState::IGNITER:
            return "IGNITER";
        case FiringSequenceState::IGNITER_WAIT:
            return "IGNITER_WAIT";
        case FiringSequenceState::PILOT_FLAME:
            return "PILOT_FLAME";
        case FiringSequenceState::PILOT_FLAME_WAIT:
            return "PILOT_FLAME_WAIT";
        case FiringSequenceState::RAMP_UP:
            return "RAMP_UP";
        case FiringSequenceState::FULL_THROTTLE:
            return "FULL_THROTTLE";
        case FiringSequenceState::LOW_THROTTLE:
            return "LOW_THROTTLE";
        case FiringSequenceState::ENDED:
            return "ENDED";
        default:
            return "UNKNOWN";
    }
}

struct FiringSequenceData
{
    uint64_t timestamp;
    FiringSequenceState state;

    FiringSequenceData() : timestamp{0}, state{FiringSequenceState::IDLE} {}

    FiringSequenceData(uint64_t timestamp, FiringSequenceState state)
        : timestamp{timestamp}, state{state}
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FiringSequenceData,
                          FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

}  // namespace Motor
