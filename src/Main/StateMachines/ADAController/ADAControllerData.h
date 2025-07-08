/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <algorithms/ADA/ADAData.h>

#include <cstdint>
#include <ostream>
#include <reflect.hpp>
#include <string>

namespace Main
{

enum class ADAControllerState : uint8_t
{
    INIT = 0,
    CALIBRATING,
    READY,
    ARMED,
    SHADOW_MODE,
    ACTIVE_ASCENT,
    ACTIVE_DROGUE_DESCENT,
    ACTIVE_TERMINAL_DESCENT,
    END
};

struct ADAControllerStatus
{
    uint64_t timestamp       = 0;
    ADAControllerState state = ADAControllerState::INIT;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADAControllerStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

struct ADAControllerSampleData
{
    uint64_t timestamp               = 0;
    uint32_t ada0DetectedApogees     = 0;
    uint32_t ada2DetectedApogees     = 0;
    uint32_t ada1DetectedApogees     = 0;
    uint32_t ada0DetectedDeployments = 0;
    uint32_t ada1DetectedDeployments = 0;
    uint32_t ada2DetectedDeployments = 0;

    ADAControllerState state = ADAControllerState::INIT;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADAControllerSampleData,
                          FIELD_DEF(timestamp) FIELD_DEF(ada0DetectedApogees)
                              FIELD_DEF(ada1DetectedApogees)
                                  FIELD_DEF(ada2DetectedApogees)
                                      FIELD_DEF(ada0DetectedDeployments)
                                          FIELD_DEF(ada1DetectedDeployments)
                                              FIELD_DEF(ada2DetectedDeployments)
                                                  FIELD_DEF(state));
    }
};

// A collection of the states of the three ada algorithms
struct ADA0State : Boardcore::ADAState
{
    ADA0State(Boardcore::ADAState state) : Boardcore::ADAState(state) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADA0State, EXTEND_DEF(Boardcore::ADAState));
    }
};

struct ADA1State : Boardcore::ADAState
{
    ADA1State(Boardcore::ADAState state) : Boardcore::ADAState(state) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADA1State, EXTEND_DEF(Boardcore::ADAState));
    }
};

struct ADA2State : Boardcore::ADAState
{
    ADA2State(Boardcore::ADAState state) : Boardcore::ADAState(state) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADA2State, EXTEND_DEF(Boardcore::ADAState));
    }
};

}  // namespace Main
