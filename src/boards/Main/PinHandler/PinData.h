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

#include <cstdint>
#include <ostream>

namespace Main
{

struct PinChangeData
{
    uint64_t timestamp    = 0;
    uint8_t pinId         = 0;
    bool lastState        = false;
    uint32_t changesCount = 0;

    PinChangeData(uint64_t timestamp, uint8_t pinId, bool lastState,
                  uint32_t changesCount)
        : timestamp(timestamp), pinId(pinId), lastState(lastState),
          changesCount(changesCount)
    {
    }

    static std::string header()
    {
        return "timestamp,pinId,lastState,changesCount\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << static_cast<int>(pinId) << ","
           << static_cast<int>(lastState) << "," << changesCount << "\n";
    }
};

}  // namespace Main