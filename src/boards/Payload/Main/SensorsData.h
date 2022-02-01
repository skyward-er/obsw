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

namespace PayloadBoard
{

struct AirSpeedPitot
{
    uint64_t timestamp;
    float airspeed;

    static std::string header() { return "timestamp,airspeed\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << airspeed << "\n";
    }
};

enum SensorDriverStatus
{
    DRIVER_ERROR = 0,
    DRIVER_OK    = 1
};

struct SensorsStatus
{
    uint8_t bmx160       = DRIVER_OK;
    uint8_t ms5803       = DRIVER_OK;
    uint8_t lis3mdl      = DRIVER_OK;
    uint8_t gps          = DRIVER_OK;
    uint8_t internal_adc = DRIVER_OK;
    uint8_t ads1118      = DRIVER_OK;

    static std::string header()
    {
        return "bmx160,ms5803,lis3mdl,gps,internal_adc,ads1118\n";
    }

    void print(std::ostream& os) const
    {
        os << (int)bmx160 << "," << (int)ms5803 << "," << (int)lis3mdl << ","
           << (int)gps << "," << (int)internal_adc << "," << (int)ads1118
           << "\n";
    }
};

}  // namespace PayloadBoard
