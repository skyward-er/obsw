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

namespace Main
{

enum FlightModeManagerState : uint8_t
{
    FMM_STATE_ON_GROUND = 0,
    FMM_STATE_INIT,
    FMM_STATE_INIT_ERROR,
    FMM_STATE_INIT_DONE,
    FMM_STATE_CALIBRATE_SENSORS,
    FMM_STATE_CALIBRATE_ALGORITHMS,
    FMM_STATE_DISARMED,
    FMM_STATE_TEST_MODE,
    FMM_STATE_ARMED,
    FMM_STATE_IGNITION,  // < Unused, kept for backward compatibility
    FMM_STATE_FLYING,
    FMM_STATE_POWERED_ASCENT,
    FMM_STATE_UNPOWERED_ASCENT,
    FMM_STATE_DROGUE_DESCENT,
    FMM_STATE_TERMINAL_DESCENT,
    FMM_STATE_LANDED,
    FMM_STATE_INVALID
};

struct FlightModeManagerStatus
{
    uint64_t timestamp           = 0;
    FlightModeManagerState state = FlightModeManagerState::FMM_STATE_INVALID;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

}  // namespace Main