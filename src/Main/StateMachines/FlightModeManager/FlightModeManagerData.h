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
#include <reflect.hpp>

namespace Main
{

enum class FlightModeManagerState : uint8_t
{
    ON_GROUND = 0,
    INIT,
    INIT_ERROR,
    INIT_DONE,
    CALIBRATE_SENSORS,
    CALIBRATE_ALGORITHMS,
    DISARMED,
    TEST_MODE,
    ARMED,
    IGNITION,  // < Unused, kept for backward compatibility
    FLYING,
    POWERED_ASCENT,
    UNPOWERED_ASCENT,
    DROGUE_DESCENT,
    TERMINAL_DESCENT,
    LANDED
};

struct FlightModeManagerStatus
{
    uint64_t timestamp           = 0;
    FlightModeManagerState state = FlightModeManagerState::ON_GROUND;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FlightModeManagerStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

}  // namespace Main
