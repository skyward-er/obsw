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

#include <algorithms/Ereg/EregData.h>

#include <cstdint>
#include <iostream>
#include <reflect.hpp>
#include <string>

namespace Motor
{

enum class EregState : uint8_t
{
    INIT = 0,
    CLOSED,
    PRESSURIZING,
    PILOTFLAME,
    FIRING,
    INVALID,
};

inline std::string to_string(EregState state)
{
    switch (state)
    {
        case EregState::INIT:
            return "INIT";
        case EregState::CLOSED:
            return "CLOSED";
        case EregState::PRESSURIZING:
            return "PRESSURIZING";
        case EregState::PILOTFLAME:
            return "PILOTFLAME";
        case EregState::FIRING:
            return "FIRING";
        default:
            return "UNKNOWN";
    }
}

struct ERegStateData
{
    uint64_t timestamp;
    EregState state;

    ERegStateData() : timestamp{0}, state{EregState::INIT} {}

    ERegStateData(uint64_t timestamp, EregState state)
        : timestamp{timestamp}, state{state}
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ERegStateData, FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

struct ERegSampleData
{
    uint64_t timestamp  = 0;
    float pressure      = 0.0f;
    float valvePosition = 0.0f;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ERegSampleData,
                          FIELD_DEF(timestamp) FIELD_DEF(pressure)
                              FIELD_DEF(valvePosition));
    }
};

struct EregOxData : public Boardcore::EregData
{
    explicit EregOxData(const Boardcore::EregData& data)
        : Boardcore::EregData(data)
    {
    }

    EregOxData(long timestamp, float downstreamPressure, float upstreamPressure,
               float filteredDownstreamPressure, float filteredUpstreamPressure,
               float servoPosition)
        : Boardcore::EregData(timestamp, downstreamPressure, upstreamPressure,
                              filteredDownstreamPressure,
                              filteredUpstreamPressure, servoPosition)
    {
    }

    EregOxData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(EregOxData, EXTEND_DEF(Boardcore::EregData));
    }
};

struct EregFuelData : public Boardcore::EregData
{
    explicit EregFuelData(const Boardcore::EregData& data)
        : Boardcore::EregData(data)
    {
    }

    EregFuelData(long timestamp, float downstreamPressure,
                 float upstreamPressure, float filteredDownstreamPressure,
                 float filteredUpstreamPressure, float servoPosition)
        : Boardcore::EregData(timestamp, downstreamPressure, upstreamPressure,
                              filteredDownstreamPressure,
                              filteredUpstreamPressure, servoPosition)
    {
    }

    EregFuelData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(EregFuelData, EXTEND_DEF(Boardcore::EregData));
    }
};

}  // namespace Motor
