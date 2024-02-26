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

#include <sensors/ADS131M08/ADS131M08Data.h>
#include <sensors/MAX31856/MAX31856Data.h>

namespace RIGv2
{
struct ADCsData : Boardcore::ADS131M08Data
{
    uint8_t adcNumber;

    ADCsData() : ADS131M08Data{0, 0, 0, 0, 0, 0, 0, 0, 0}, adcNumber{0} {}

    ADCsData(uint64_t time, uint8_t num, float ch1, float ch2, float ch3,
             float ch4, float ch5, float ch6, float ch7, float ch8)
        : ADS131M08Data{time, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8},
          adcNumber{num}
    {
    }

    static std::string header()
    {
        return "timestamp,adcNumber,voltage_channel_1,voltage_channel_2,"
               "voltage_channel_3,voltage_channel_4,voltage_channel_5,voltage_"
               "channel_6,voltage_channel_7,voltage_channel_8\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)adcNumber << "," << voltage[0] << ","
           << voltage[1] << "," << voltage[2] << "," << voltage[3] << ","
           << voltage[4] << "," << voltage[5] << "," << voltage[6] << ","
           << voltage[7] << "\n";
    }
};

struct TCsData : Boardcore::MAX31856Data
{
    uint8_t tcNumber;

    TCsData() : MAX31856Data{0, 0}, tcNumber{0} {}

    TCsData(uint64_t time, uint8_t num, float temperature,
            float coldJunctionTemperature)
        : MAX31856Data{time, temperature, coldJunctionTemperature},
          tcNumber{num}
    {
    }

    static std::string header()
    {
        return "temperatureTimestamp,tcNumber,temperature,"
               "coldJunctionTemperature\n";
    }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << (int)tcNumber << "," << temperature
           << "," << coldJunctionTemperature << "\n";
    }
};
}  // namespace RIGv2