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

#include "DipStatus.h"

#include <drivers/DipSwitch/DipSwitch.h>
#include <interfaces-impl/bsp_impl.h>
#include <miosix.h>

using namespace Boardcore;
using namespace miosix;

namespace Groundstation
{

DipStatus readDipStatus()
{
    uint32_t microSecClk = 100;
    GpioPin sh           = dipSwitch::sh::getPin();
    GpioPin clk          = dipSwitch::clk::getPin();
    GpioPin qh           = dipSwitch::qh::getPin();

    DipSwitch dip(sh, clk, qh, std::chrono::microseconds(microSecClk));
    uint8_t read = dip.read();

    DipStatus dipRead;
    dipRead.isARP            = 1 & read;
    dipRead.mainHasBackup    = 1 & (read >> 1);
    dipRead.payloadHasBackup = 1 & (read >> 2);
    dipRead.mainTXenable     = 1 & (read >> 3);
    dipRead.payloadTXenable  = 1 & (read >> 4);
    dipRead.ipConfig         = read >> 5;
    return dipRead;
}

}  // namespace Groundstation
