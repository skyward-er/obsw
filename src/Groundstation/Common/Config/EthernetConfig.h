/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <drivers/WIZ5500/WIZ5500.h>

#include <cstdint>

namespace Groundstation
{

namespace detail
{
constexpr auto nextSubnetwork(Boardcore::WizIp ip)
{
    ip.c += 1;
    return ip;
}

constexpr auto nextMacGroup(Boardcore::WizMac mac)
{
    mac.d += 1;
    return mac;
}
}  // namespace detail

constexpr uint16_t RECV_PORT = 42070;
constexpr uint16_t SEND_PORT = 42069;

constexpr Boardcore::WizMac MAC_BASE = {0x69, 0x69, 0x69, 0x69, 0, 0};
constexpr Boardcore::WizIp IP_BASE   = {169, 254, 1, 0};
constexpr Boardcore::WizIp GATEWAY   = {169, 254, 1, 1};
constexpr Boardcore::WizIp SUBNET    = {255, 255, 0, 0};

// Base addresses used when static IP is enabled
constexpr Boardcore::WizIp GS_IP_BASE    = IP_BASE;
constexpr Boardcore::WizMac GS_MAC_BASE  = MAC_BASE;
constexpr Boardcore::WizIp RIG_IP_BASE   = detail::nextSubnetwork(GS_IP_BASE);
constexpr Boardcore::WizMac RIG_MAC_BASE = detail::nextMacGroup(GS_MAC_BASE);

}  // namespace Groundstation
