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

#include <events/Event.h>

namespace Biliquid
{
enum Events : Boardcore::Event
{
    START_SEQUENCE_1 = Boardcore::BasicEvent::EV_FIRST_CUSTOM,
    START_SEQUENCE_2,
    START_SEQUENCE_3,
    STOP_SEQUENCE_1,
    STOP_SEQUENCE_2,
    STOP_SEQUENCE_3,
    CONTINUE_SEQUENCE_1,
    CONTINUE_SEQUENCE_2,
    CONTINUE_SEQUENCE_3,
};

inline const char* eventToString(Boardcore::Event ev)
{
    switch (ev)
    {
        case Events::START_SEQUENCE_1:
            return "START_SEQUENCE_1";
        case Events::START_SEQUENCE_2:
            return "START_SEQUENCE_2";
        case Events::START_SEQUENCE_3:
            return "START_SEQUENCE_3";
        case Events::STOP_SEQUENCE_1:
            return "STOP_SEQUENCE_1";
        case Events::STOP_SEQUENCE_2:
            return "STOP_SEQUENCE_2";
        case Events::STOP_SEQUENCE_3:
            return "STOP_SEQUENCE_3";
        case Events::CONTINUE_SEQUENCE_1:
            return "CONTINUE_SEQUENCE_1";
        case Events::CONTINUE_SEQUENCE_2:
            return "CONTINUE_SEQUENCE_2";
        case Events::CONTINUE_SEQUENCE_3:
            return "CONTINUE_SEQUENCE_3";
        default:
            return "UNKNOWN_EVENT";
    }
}

enum Topics : uint8_t
{
    CONTROL_SEQUENCE,
};

}  // namespace Biliquid
