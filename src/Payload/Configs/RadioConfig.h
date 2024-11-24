/* Copyright (c) 2024 Skyward Experimental Rocketry
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
#pragma once

#include <common/MavlinkLyra.h>
#include <units/Frequency.h>

#include <chrono>

namespace Payload
{
namespace Config
{
namespace Radio
{

/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace std::chrono_literals;

constexpr auto LOW_RATE_TELEMETRY  = 2_hz;
constexpr auto HIGH_RATE_TELEMETRY = 4_hz;
constexpr auto MESSAGE_QUEUE_SIZE  = 30;

constexpr bool MAVLINK_OVER_HIL_SERIAL_ENABLED = true;

namespace Mavlink
{
constexpr uint8_t SYSTEM_ID    = SysIDs::MAV_SYSID_PAYLOAD;
constexpr uint8_t COMPONENT_ID = 0;
}  // namespace Mavlink

namespace MavlinkDriver
{
constexpr auto PKT_LENGTH       = 255;
constexpr auto PKT_QUEUE_SIZE   = 20;
constexpr auto MSG_LENGTH       = MAVLINK_MAX_DIALECT_PAYLOAD_SIZE;
constexpr auto SLEEP_AFTER_SEND = 0ms;
constexpr auto MAX_PKT_AGE      = 10ms;
}  // namespace MavlinkDriver

}  // namespace Radio
}  // namespace Config
}  // namespace Payload
