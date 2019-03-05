/* 
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once
#include <type_traits>

template <typename T>
struct SensorStat
{
    // T must be an arithmetic type
    static_assert(std::is_arithmetic(T));

    unit32_t n_samples = 0;

    T mean()
    {
        return M;
    }

    T variance()
    {
        return S / (n_samples - 1);
    }

    void update(T sample)
    {
        ++n_samples;
        T old_m = m;

        m = m + (sample - m)/n_samples;
        S = s + (sample - old_m)*(sample - m);
    }
private:
    // Running mean and variance
    T m = 0;
    T s = 0;
};