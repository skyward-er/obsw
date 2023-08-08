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

#include <cstddef>
#include <cstdint>

// Uncomment the following line to enable backup RF for main
// #define SKYWARD_GS_MAIN_USE_BACKUP_RF
// Uncomment the following line to enable backup RF for payload
// #define SKYWARD_GS_PAYLOAD_USE_BACKUP_RF

namespace Groundstation
{

constexpr size_t MAV_OUT_QUEUE_SIZE         = 10;
constexpr size_t MAV_PENDING_OUT_QUEUE_SIZE = 10;
constexpr uint16_t MAV_SLEEP_AFTER_SEND     = 0;
constexpr size_t MAV_OUT_BUFFER_MAX_AGE     = 10;

/// @brief Every how many ms force the flush of the send queue.
constexpr unsigned int AUTOMATIC_FLUSH_PERIOD = 250;
/// @brief After how many ms stop waiting for the other side to send commands.
constexpr long long AUTOMATIC_FLUSH_DELAY = 2000;

/// @brief Period of the radio status telemetry.
constexpr unsigned int RADIO_STATUS_PERIOD = 250;
/// @brief Size in ms of the radio moving bitrate window size.
constexpr size_t RADIO_BITRATE_WINDOW_SIZE = 1000;

}  // namespace Groundstation
