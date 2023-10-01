/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <sensors/ADS131M04/ADS131M04Data.h>

namespace RIG
{
struct ADCsData : Boardcore::ADS131M04Data
{
    uint8_t ADCnumber;

    ADCsData() : ADS131M04Data{0, 0, 0, 0, 0} { ADCnumber = 0; }

    ADCsData(uint64_t time, uint8_t num, float ch1, float ch2, float ch3,
             float ch4)
        : ADS131M04Data{time, ch1, ch2, ch3, ch4}
    {
        ADCnumber = num;
    }

    static std::string header()
    {
        return "timestamp,ADCnumber,voltage_channel_1,voltage_channel_2,"
               "voltage_channel_"
               "3,voltage_channel_4\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)ADCnumber << "," << voltage[0] << ","
           << voltage[1] << "," << voltage[2] << "," << voltage[3] << "\n";
    }
};
}  // namespace RIG