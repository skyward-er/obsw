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

#include <algorithms/ANAS/ANASData.h>
#include <algorithms/NASDAQ/NASDAQData.h>

#include <cstdint>
#include <ostream>
#include <reflect.hpp>
#include <string>

namespace Main
{

enum class NASControllerState : uint8_t
{
    INIT = 0,
    CALIBRATING,
    READY,
    ACTIVE_ASCENT,
    DESCENT,
    END
};

struct NASControllerStatus
{
    uint64_t timestamp       = 0;
    NASControllerState state = NASControllerState::INIT;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(NASControllerStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

struct NASState
{
    uint64_t timestamp = 0;

    float n = 0;  ///< North (x)
    float e = 0;  ///< East  (y)
    float d = 0;  ///< Down  (z)

    float vn = 0;  ///< Velocity North (x)
    float ve = 0;  ///< Velocity East  (y)
    float vd = 0;  ///< Velocity Down  (z)

    NASState() : timestamp(0) {};

    NASState(Boardcore::NASDAQState nasdaqState)
    {
        timestamp = nasdaqState.timestamp;
        n         = nasdaqState.n;
        e         = nasdaqState.e;
        d         = nasdaqState.d;
        vn        = nasdaqState.vn;
        ve        = nasdaqState.ve;
        vd        = nasdaqState.vd;
    }

    NASState(Boardcore::ANASState anasState)
    {
        timestamp = anasState.timestamp;
        n         = anasState.n;
        e         = anasState.e;
        d         = anasState.d;
        vn        = anasState.vn;
        ve        = anasState.ve;
        vd        = anasState.vd;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(NASState, FIELD_DEF(timestamp) FIELD_DEF(n)
                                        FIELD_DEF(e) FIELD_DEF(d) FIELD_DEF(vn)
                                            FIELD_DEF(ve) FIELD_DEF(vd));
    }
};

}  // namespace Main
