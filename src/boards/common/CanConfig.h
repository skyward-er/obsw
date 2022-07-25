/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

namespace Common
{

namespace CanConfig
{

static constexpr uint32_t BAUD_RATE = 500 * 1000;
static constexpr float SAMPLE_POINT = 87.5f / 100.0f;

enum class Priority : uint8_t
{
    CRITICAL = 0,
    HIGH,
    MEDIUM,
    LOW
};

enum class PrimaryType : uint8_t
{
    EVENTS = 0,
    SENSORS
};

/// Used for source and destination
enum class Board : uint8_t
{
    BROADCAST = 0,
    MAIN,
    PAYLOAD,
    AUXILIARY
};

enum class SensorId : uint8_t
{
    PITOT
};

enum class EventId : uint8_t
{
    LIFTOFF = 0,
    APOGEE,
    ARM,
    DISARM,
    CAM_ON,
    CAM_OFF
};

}  // namespace CanConfig

}  // namespace Common
