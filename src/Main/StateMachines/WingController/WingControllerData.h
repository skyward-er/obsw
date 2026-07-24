/* Copyright (c) 2026 Skyward Experimental Rocketry
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

#include <prf/PRF_types.h>

#include <cstdint>
#include <iostream>
#include <reflect.hpp>
#include <string>

namespace Main
{

enum class WingControllerState : uint8_t
{
    INIT = 0,
    READY,
    DEPLOYMENT,
    OPENING_PUMPS_PULL,
    OPENING_PUMPS_RELEASE,
    GUIDED_DESCENT,
    LANDING_FLARE,
    LANDED,
};

struct WingControllerStatus
{
    uint64_t timestamp        = 0;
    WingControllerState state = WingControllerState::INIT;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(WingControllerStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

struct WingControllerLogsData
{
    uint64_t timestamp;
    PRF_types_h_::PRFLogs PRFLogs;

    WingControllerLogsData() : timestamp(0), PRFLogs() {};

    WingControllerLogsData(uint64_t timestamp,
                           PRF_types_h_::PRFLogs wingControllerLogs)
        : timestamp(timestamp), PRFLogs(wingControllerLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            WingControllerLogsData,
            FIELD_DEF(timestamp) FIELD_DEF2(PRFLogs, Q1) FIELD_DEF2(PRFLogs, Q2)
                FIELD_DEF2(PRFLogs, TerminalTarget) FIELD_DEF2(
                    PRFLogs, TargetIndex) FIELD_DEF2(PRFLogs, Heading)
                    FIELD_DEF2(PRFLogs, Reference)
                        FIELD_DEF2(PRFLogs, ServoCommands)
                            FIELD_DEF2(PRFLogs, WindHeading));
    }
};

struct FlareCommandData
{
    uint64_t timestamp = 0;
    float angleLeft    = 0;
    float angleRight   = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FlareCommandData,
                          FIELD_DEF(timestamp) FIELD_DEF(angleLeft)
                              FIELD_DEF(angleRight));
    }
};

}  // namespace Main
