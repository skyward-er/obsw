/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Common.h>
#include <DeathStack/configs/TMTCConfig.h>
#include <DeathStack/Status.h>

namespace DeathStackBoard
{
namespace TMBuilder
{

/**
 * Synchronously read the corresponding telemetry from the statusRepo 
 * (aka last logged struct).
 * @param req_tm     requested telemetry
 * @return           the telemetry as mavlink_message_t
 */
static mavlink_message_t buildTelemetry(uint8_t req_tm) 
{
	miosix::PauseKernelLock kLock;
    return Status::getTM(req_tm, TMTC_MAV_SYSID, TMTC_MAV_SYSID);
}

} /* namespace TMBuilder */
} /* namespace DeathStackBoard */
