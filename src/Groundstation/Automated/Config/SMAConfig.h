/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli, Nicolò Caruso
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
#include <chrono>

/* linter off */ using namespace std::chrono;

namespace Antennas
{

namespace SMAConfig
{

/// @brief Period of the propagator algorithm [ms].
constexpr milliseconds UPDATE_PERIOD = 100ms;  // 10 Hz

// No feedback gains for the Follower
static constexpr float YAW_GAIN_NF =
    1.0;  ///< Yaw gain for the no feedback states
static constexpr float PITCH_GAIN_NF =
    1.0;  ///< Pitch gain for the no feedback states

// Feedback gains for the Follower
static constexpr float YAW_GAIN_F = 0.1;  ///< Yaw gain for the feedback states
static constexpr float PITCH_GAIN_F =
    1.0;  ///< Pitch gain for the feedback states

}  // namespace SMAConfig
}  // namespace Antennas
