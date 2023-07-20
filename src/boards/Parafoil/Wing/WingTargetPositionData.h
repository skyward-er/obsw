/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Radu Raul
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

namespace Parafoil
{

struct WingTargetPositionData
{
    float receivedLat;
    float receivedLon;
    float latitude;
    float longitude;
    float emcLat;
    float emcLon;
    float m1Lat;
    float m1Lon;
    float m2Lat;
    float m2Lon;

    float n;
    float e;

    static std::string header()
    {
        return "receivedLat, receivedLon, "
               "latitude,longitude,n,e,EMCLat,EMCLon,M1Lat,M1Lon,M2Lat,M2Lon\n";
    }

    void print(std::ostream& os) const
    {
        os << receivedLat << "," << receivedLon << "," << latitude << ","
           << longitude << "," << n << "," << e << "," << emcLat << ","
           << emcLon << "," << m1Lat << "," << m1Lon << "," << m2Lat << ","
           << m2Lon << "\n";
    }
};

}  // namespace Parafoil
