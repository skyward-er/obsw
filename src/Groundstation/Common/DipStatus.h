/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Domenicali
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

namespace Groundstation
{

/**
 * @brief Dip switch status for the GS board.
 *
 * Dipswitch configuration
 *  arp mb  pb  mtx ptx ip3 ip2 ip1
 * | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
 * ---------------------------------
 * | I | I | I | I | I | I | I | I |
 * | O | O | O | O | O | O | O | O |
 * ---------------------------------
 * | H | G | F | E | D | C | B | A |
 */
struct DipStatus
{
    bool isARP;
    bool mainHasBackup;
    bool payloadHasBackup;
    bool mainTXenable;
    bool payloadTXenable;
    uint8_t ipConfig;

    void print(std::ostream& os) const
    {
        os << "Dipswitch state:"
           << "\n\tARP mode:             " << isARP
           << "\n\tMain backup radio:    " << mainHasBackup
           << "\n\tPayload backup radio: " << payloadHasBackup
           << "\n\tMain TX enabled:      " << mainTXenable
           << "\n\tPayload TX enabled:   " << payloadTXenable
           << "\n\tIP offset:            " << (int)ipConfig << std::endl;
    }
};

/// Reads the physical dip switch and decodes it into a DipStatus.
DipStatus readDipStatus();

}  // namespace Groundstation
