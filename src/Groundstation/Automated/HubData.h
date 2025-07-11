/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include <iostream>
#include <reflect.hpp>
#include <string>

namespace Antennas
{

/**
 * @brief Structure to save informations about the Hub reception
 */
struct HubData
{
    uint64_t timestamp = 0;
    uint16_t groundRx  = 0;
    uint16_t rocketRx  = 0;
    uint16_t sniffedRx = 0;
    float cpuMean      = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(HubData, FIELD_DEF(timestamp) FIELD_DEF(groundRx)
                                       FIELD_DEF(rocketRx) FIELD_DEF(sniffedRx)
                                           FIELD_DEF(cpuMean));
    }
};
}  // namespace Antennas
