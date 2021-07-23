/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

namespace DeathStackBoard
{
typedef struct
{
    float n000 = 0.496878;
    float n100 = -1.405038;
    float n200 = 6.502618;
    float n300 = -18.314261;
    float n400 = 30.152448;
    float n500 = -26.715700;
    float n600 = 9.711730;
    float n010 = 8.486901;
    float n020 = 141.050398;
    float n110 = 1.233043;
    float n120 = -152.610100;
    float n210 = 81.980768;
    float n220 = 1072.170007;
    float n310 = -309.620754;
    float n320 = -3618.989164;
    float n410 = 455.524477;
    float n420 = 5190.202262;
    float n510 = -212.545170;
    float n520 = -2402.939515;
    float n001 = 0.000003;
} coeffs_t;

const coeffs_t coeffs;
}  // namespace DeathStackBoard
