/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <common/MavlinkGemini.h>

namespace ConRIG
{

namespace Config
{
namespace Radio
{

static constexpr unsigned int MAV_OUT_QUEUE_SIZE = 20;
static constexpr unsigned int MAV_MAX_LENGTH = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;

static constexpr uint16_t MAV_SLEEP_AFTER_SEND = 0;
static constexpr size_t MAV_OUT_BUFFER_MAX_AGE = 10;

// Mavlink ids
static constexpr uint8_t MAV_SYSTEM_ID    = 171;
static constexpr uint8_t MAV_COMPONENT_ID = 96;

// Periodic telemetries frequency
constexpr uint32_t PING_GSE_PERIOD = 500;  // [ms]

static constexpr uint32_t MAVLINK_QUEUE_SIZE = 3;

}  // namespace Radio
}  // namespace Config
}  // namespace ConRIG
