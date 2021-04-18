/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#include <interfaces-impl/hwmapping.h>
#include <skyward-boardcore/libs/mavlink_skyward_lib/mavlink_lib/hermes/mavlink.h>

#include "drivers/Xbee/Xbee.h"
#include "drivers/spi/SPIDriver.h"

namespace DeathStackBoard
{

/* Mavlink Driver queue settings */
static constexpr unsigned int MAV_OUT_QUEUE_LEN   = 10;
static constexpr unsigned int MAV_PKT_SIZE        = 255;
static constexpr long long MAV_OUT_BUFFER_MAX_AGE = 200;
/* Min guaranteed sleep time after each packet sent(milliseconds) */
static const uint16_t TMTC_SLEEP_AFTER_SEND = 0;

/* Xbee */
typedef miosix::xbee::cs XbeeCS;
typedef miosix::xbee::attn XbeeATTN;
typedef miosix::xbee::reset XbeeRST;

/* Periodic telemetries periods */
static const unsigned int LR_TM_TIMEOUT = 1000;
static const unsigned int HR_TM_TIMEOUT = 63;

static const unsigned int TEST_TM_TIMEOUT = 250;

/* Mavlink messages sysID and compID */
static const unsigned int TMTC_MAV_SYSID  = 1;
static const unsigned int TMTC_MAV_COMPID = 1;

}  // namespace DeathStackBoard
