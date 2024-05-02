/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <interfaces-impl/hwmapping.h>

namespace Main
{
namespace Config
{
namespace Actuators
{
static constexpr unsigned int ABK_MIN_PULSE = 1950;
static constexpr unsigned int ABK_MAX_PULSE = 1380;

// Inverted to invert the servo logic
static constexpr unsigned int EXP_MIN_PULSE = 900;
static constexpr unsigned int EXP_MAX_PULSE = 2000;

// Buzzer configs
static constexpr uint16_t BUZZER_FREQUENCY = 1000;
static constexpr float BUZZER_DUTY_CYCLE   = 0.5;
}  // namespace Actuators
}  // namespace Config
}  // namespace Main