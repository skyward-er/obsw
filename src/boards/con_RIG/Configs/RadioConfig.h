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

#include <common/Mavlink.h>

namespace con_RIG
{

namespace Config
{
namespace Radio
{

// Mavlink driver template parameters
constexpr uint32_t RADIO_PKT_LENGTH     = 255;
constexpr uint32_t RADIO_OUT_QUEUE_SIZE = 10;
constexpr uint32_t RADIO_MAV_MSG_LENGTH = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;

// Mavlink driver parameters
constexpr uint16_t MAV_SLEEP_AFTER_SEND = 10;
constexpr uint8_t MAV_PING_MSG_ID       = MAVLINK_MSG_ID_CONRIG_STATE_TC;

// Mavlink ids
constexpr uint8_t MAV_SYSTEM_ID    = 171;
constexpr uint8_t MAV_COMPONENT_ID = 96;

// Periodic telemetries frequency
constexpr uint32_t PING_GSE_PERIOD = 500;  // [ms]

// Periodic telemetries tasks ids
constexpr uint8_t PING_GSE_TASK_ID = 200;

constexpr uint32_t MAVLINK_QUEUE_SIZE = 3;

}  // namespace Radio
}  // namespace Config
}  // namespace con_RIG
