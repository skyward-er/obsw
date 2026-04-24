/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <units/Frequency.h>

#include <chrono>
namespace Motor
{
namespace Config
{
namespace FiringSequence
{

using namespace std::chrono;
using namespace Boardcore::Units::Frequency;

static constexpr Hertz UPDATE_RATE = 10_hz;

static constexpr milliseconds IGN_OX_OPENING_TIME{2500};
static constexpr milliseconds IGN_FUEL_DELAY{100};
static constexpr milliseconds IGN_FUEL_OPENING_TIME{2000};
static constexpr milliseconds SPARK_TIME{500};

static constexpr milliseconds PILOT_FLAME_MAX_TIME{2000};
static constexpr float MAIN_CHAMBER_SAFETY_THRESHOLD = 35.0f * 1.5f;  // bar

static constexpr milliseconds RAMP_UP_OPENING_TIME{500};
static constexpr milliseconds FULL_THROTTLE_TIME{3000};
static constexpr milliseconds LOW_THROTTLE_TIME{1000};

// placeholder values
static constexpr float PILOT_OX_POSITION          = 0.5f;
static constexpr float PILOT_FUEL_POSITION        = 0.5f;
static constexpr float LOW_THROTTLE_OX_POSITION   = 0.6f;
static constexpr float LOW_THROTTLE_FUEL_POSITION = 0.6f;

static constexpr float IGNITER_PRESSURE_THRESHOLD         = -500.0f;  // bar
static constexpr uint8_t IGNITER_CONFIRMATION_SAMPLES     = 20;
static constexpr float PILOT_FLAME_PRESSURE_THRESHOLD     = -500.0f;  // bar
static constexpr uint8_t PILOT_FLAME_CONFIRMATION_SAMPLES = 20;

}  // namespace FiringSequence
}  // namespace Config
}  // namespace Motor

