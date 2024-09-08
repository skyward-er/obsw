/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Mandelli, Niccolò Betto
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

namespace Payload
{

enum class WingControllerState : uint8_t
{
    UNINIT = 0,
    IDLE,
    FLYING_CALIBRATION,
    FLYING_CONTROLLED_DESCENT,
    ON_GROUND,
    END
};

struct WingControllerStatus
{
    uint64_t timestamp        = 0;
    WingControllerState state = WingControllerState::UNINIT;

    static std::string header() { return "timestamp,state\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "\n";
    }
};

struct WingControllerAlgorithmData
{
    uint64_t timestamp = 0;
    uint8_t algorithm  = 0;

    static std::string header() { return "timestamp,algorithm\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)algorithm << "\n";
    }
};

}  // namespace Payload
