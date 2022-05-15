/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

namespace Payload
{
// TODO Define PITOT

enum SensorDriverStatus
{
    DRIVER_ERROR = 0,
    DRIVER_OK    = 1
};

struct SensorsStatus
{
    uint8_t BMX160      = DRIVER_OK;
    uint8_t MS5803      = DRIVER_OK;
    uint8_t LIS3MDL     = DRIVER_OK;
    uint8_t GPS         = DRIVER_OK;
    uint8_t InternalADC = DRIVER_OK;
    uint8_t ADS1118     = DRIVER_OK;

    static std::string header()
    {
        return "BMX160,MS5803,LIS3MDL,GPS,InternalADC,ADS1118\n";
    }

    void print(std::ostream& os) const
    {
        os << (int)BMX160 << "," << (int)MS5803 << "," << (int)LIS3MDL << ","
           << (int)GPS << "," << (int)InternalADC << "," << (int)ADS1118
           << "\n";
    }
};
}  // namespace Payload