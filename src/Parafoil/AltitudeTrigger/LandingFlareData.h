/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Raul Radu
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

namespace Parafoil
{

struct LandingFlareData
{
    uint64_t timestamp        = 0;
    bool flare_detected = 0;
    float detection_altitude = 0;
    float estimated_agl_u = 0;
    float map_n = 0;
    float map_e = 0;
    float map_u = 0;
    static constexpr auto reflect()
    {
        return STRUCT_DEF(LandingFlareData,
                          FIELD_DEF(timestamp) FIELD_DEF(flare_detected) FIELD_DEF(detection_altitude) FIELD_DEF(estimated_agl_u) FIELD_DEF(map_n) FIELD_DEF(map_e) FIELD_DEF(map_u));
    }
};

}  // namespace Parafoil
