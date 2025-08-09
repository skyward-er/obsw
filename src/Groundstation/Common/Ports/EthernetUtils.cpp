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

#include "EthernetUtils.h"

#include <cstdlib>

using namespace Boardcore;

namespace Groundstation
{
WizIp generateRandomIpAddress(Boardcore::WizIp base, Boardcore::WizIp subnet)
{
    uint32_t mask      = static_cast<uint32_t>(subnet);
    uint32_t network   = static_cast<uint32_t>(base) & mask;
    uint32_t broadcast = network | ~mask;

    uint32_t rangeStart = network + 1;    // Start after the network address
    uint32_t rangeEnd   = broadcast - 1;  // End before the broadcast address

    // If the range is invalid, return the base IP
    if (rangeEnd <= rangeStart)
        return WizIp(rangeStart);

    uint32_t ip = rangeStart + (std::rand() % (rangeEnd - rangeStart + 1));

    return WizIp(ip);
}

WizMac generateRandomMacAddress(Boardcore::WizMac base)
{
    WizMac mac = base;
    mac.e      = (std::rand() % 254) + 1;  // Generate in range 1-254
    mac.f      = (std::rand() % 254) + 1;  // Generate in range 1-254

    return mac;
}
}  // namespace Groundstation
