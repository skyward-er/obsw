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

#include <drivers/BusTemplate.h>
#include <interfaces-impl/hwmapping.h>
#include <skyward-boardcore/libs/mavlink_skyward_lib/mavlink_lib/hermes/mavlink.h>
#include "drivers/Xbee/Xbee.h"

namespace DeathStackBoard
{

static const unsigned int GS_OFFLINE_TIMEOUT = 1000;  // CHANGED

/* Minimum sleep time between sends */
static const unsigned int TMTC_MIN_GUARANTEED_SLEEP = 250; // sleep between sends
static constexpr size_t MAV_OUT_BUFFER_SIZE         = 256;
static constexpr long long MAV_OUT_BUFFER_MAX_AGE   = 1000;

// SPI 2 does not work on stm32f4 discovery, use SPI1
typedef BusSPI<2, miosix::interfaces::spi2::mosi,
               miosix::interfaces::spi2::miso, miosix::interfaces::spi2::sck>
    busSPI2;

typedef Xbee::Xbee<busSPI2, miosix::xbee::cs, miosix::xbee::attn,
                   miosix::xbee::reset>
    Xbee_t;

/* Periodic telemetries periods */
static const unsigned int LR_TM_TIMEOUT = 10000;
static const unsigned int HR_TM_TIMEOUT = 200;

/* Mavlink messages sysID and compID */
static const unsigned int TMTC_MAV_SYSID  = 1;
static const unsigned int TMTC_MAV_COMPID = 1;

/* Min guaranteed sleep time after a message is sent (milliseconds) */
static const uint16_t TMTC_SLEEP_AFTER_SEND = 250;
/* Delay that estimates the send time of each byte */
static const uint16_t TMTC_SEND_MULTIPLIER = 10;
}  // namespace DeathStackBoard
