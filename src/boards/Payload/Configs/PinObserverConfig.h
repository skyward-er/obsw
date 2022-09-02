/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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
#include <miosix.h>
#include <utils/PinObserver/PinObserver.h>

namespace Payload
{

constexpr unsigned int LAUNCH_PIN_THRESHOLD = 10;
constexpr Boardcore::PinTransition LAUNCH_PIN_TRIGGER =
    Boardcore::PinTransition::FALLING_EDGE;

constexpr unsigned int NC_DETACH_PIN_THRESHOLD = 10;
constexpr Boardcore::PinTransition NC_DETACH_PIN_TRIGGER =
    Boardcore::PinTransition::FALLING_EDGE;

constexpr unsigned int DPL_SERVO_PIN_THRESHOLD = 10;
constexpr Boardcore::PinTransition DPL_SERVO_PIN_TRIGGER =
    Boardcore::PinTransition::FALLING_EDGE;

}  // namespace Payload
