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

#include <chrono>

namespace RIGv2
{
namespace Config
{
namespace Biliquid
{
/* linter off */ using namespace std::chrono;

// sequence 1 configs
static constexpr int maxStepCount                = 5;
static constexpr float PositionsOX[maxStepCount] = {0.20f, 0.25f, 0.30f, 0.35f,
                                                    0.40f};

static constexpr float PositionsFUEL[maxStepCount] = {0.15f, 0.20f, 0.25f,
                                                      0.30f, 0.35f};

// time to wait between steps in sequence 1
static constexpr milliseconds DT{1500};

// sequence 2 configs
static constexpr float SEQ_2_FUEL_POSITION = 0.25883f;
static constexpr float SEQ_2_OX_POSITION   = 0.32516f;
static constexpr milliseconds SEQ_2_OX_DELAY{100};
static constexpr milliseconds SEQ_2_SHUTDOWN_DELAY{2500};

// sequence 3 configs
static constexpr milliseconds SEQ_3_SHUTDOWN_DELAY{1500};

}  // namespace Biliquid
}  // namespace Config
}  // namespace RIGv2

