/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <cstdint>
#include <iostream>
#include <reflect.hpp>
#include <string>

namespace RIGv2
{

enum class Tars1Action : uint8_t
{
    READY = 0,
    START,
    WASHING,
    FILLING,
    VENTING,
    AUTOMATIC_STOP,
    MANUAL_STOP,
    MANUAL_ACTION_STOP,
};

struct Tars1ActionData
{
    uint64_t timestamp = 0;
    Tars1Action action = Tars1Action::READY;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Tars1ActionData,
                          FIELD_DEF(timestamp) FIELD_DEF(action));
    }
};

struct Tars1SampleData
{
    uint64_t timestamp = 0;
    float pressure     = 0.0f;
    float mass         = 0.0f;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Tars1SampleData, FIELD_DEF(timestamp) FIELD_DEF(
                                               pressure) FIELD_DEF(mass));
    }
};

}  // namespace RIGv2
