/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

// Ignore warnings as these are auto-generated headers made with third party
// tools
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink_skyward_lib/mavlink_lib/lynx/mavlink.h>
#pragma GCC diagnostic pop

#include <drivers/mavlink/MavlinkDriver.h>

namespace DeathStackBoard
{
using MavDriver = MavlinkDriver<256, 5>;
}