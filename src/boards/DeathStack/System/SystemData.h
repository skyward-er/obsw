/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <ostream>
#include <string>

struct SystemData
{
    long long timestamp  = 0;
    float cpu_usage      = 0.0f;
    float cpu_usage_min  = 0.0f;
    float cpu_usage_max  = 0.0f;
    float cpu_usage_mean = 0.0f;

    float free_heap     = 0.0f;
    float min_free_heap = 0.0f;

    static std::string header()
    {
        return "timestamp,cpu_usage,cpu_usage_min,cpu_usage_max,cpu_usage_mean,"
               "free_heap,min_free_heap\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << cpu_usage << "," << cpu_usage_min << ","
           << cpu_usage_max << "," << cpu_usage_mean << "," << free_heap << ","
           << min_free_heap << "\n";
    }
};
