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
#include <string>

namespace Antennas
{

/**
 * @brief Structure to handle the APR coordinates, a reduced set of the GPS
 * data. ARP is assumed fixed, therefore do not move.
 */
struct ARPCoordinates
{
    uint64_t gpsTimestamp = 0;
    float latitude        = 0;  // [deg]
    float longitude       = 0;  // [deg]
    float height          = 0;  // [m]
    uint8_t satellites    = 0;  // [1]
    uint8_t fix           = 0;  // 0 = no fix

    static std::string header()
    {
        return "timestamp,latitude,longitude,height,satellites,fix\n";
    }

    void print(std::ostream& os) const
    {
        os << gpsTimestamp << "," << latitude << "," << longitude << ","
           << height << "," << (int)satellites << "," << (int)fix << "\n";
    }
};
}  // namespace Antennas
