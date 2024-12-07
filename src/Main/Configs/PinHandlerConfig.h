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

#include <utils/PinObserver/PinObserver.h>

namespace Main
{

namespace Config
{

namespace PinHandler
{

constexpr uint32_t RAMP_PIN_THRESHOLD = 20;
constexpr Boardcore::PinTransition RAMP_PIN_TRIGGER =
    Boardcore::PinTransition::RISING_EDGE;

constexpr uint32_t MAIN_DETACH_PIN_THRESHOLD = 20;
constexpr Boardcore::PinTransition MAIN_DETACH_PIN_TRIGGER =
    Boardcore::PinTransition::RISING_EDGE;

constexpr uint32_t PAYLOAD_DETACH_PIN_THRESHOLD  = 20;
constexpr uint32_t EXPULSION_SENSE_PIN_THRESHOLD = 20;
constexpr uint32_t CUTTER_SENSE_PIN_THRESHOLD    = 20;

}  // namespace PinHandler

}  // namespace Config

}  // namespace Main
