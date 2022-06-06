/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Luca Conterio, Matteo Pignataro
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

#include <stdint.h>

#include <cstdio>

namespace Payload
{

// TODO: update with the correct values
static const uint32_t HR_GROUND_UPDATE_PERIOD = 62;  // [ms]
static const uint32_t HR_FLIGHT_UPDATE_PERIOD = 10;
static const uint32_t LR_UPDATE_PERIOD        = 100;  // [ms]
static const uint32_t SD_UPDATE_PERIOD        = 10000;

// TODO: define the correct ids for task scheduler
static const uint8_t RADIO_HR_ID  = 200;
static const uint8_t RADIO_LR_ID  = 201;
static const uint8_t SD_UPDATE_ID = 202;

// Mavlink Driver queue settings
static constexpr unsigned int MAV_OUT_QUEUE_LEN = 10;
static constexpr unsigned int MAV_PKT_SIZE      = 63;
static constexpr size_t MAV_OUT_BUFFER_MAX_AGE  = 200;

// These two values are taken as is
static const unsigned int TMTC_MAV_SYSID  = 171;
static const unsigned int TMTC_MAV_COMPID = 96;

// Min guaranteed sleep time after each packet sent
static const uint16_t SLEEP_AFTER_SEND = 0;  // [ms]

static const bool XBEE_80KBPS_DATA_RATE = true;
static const int XBEE_TIMEOUT           = 5000;  // [ms]

}  // namespace Payload
