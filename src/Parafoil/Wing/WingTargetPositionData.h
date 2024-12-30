/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <units/Length.h>

#include <ostream>
#include <string>

namespace Parafoil
{

struct WingTargetPositionData
{
    float targetLat = 0;
    float targetLon = 0;
    float targetN   = 0;
    float targetE   = 0;
    float emcN      = 0;
    float emcE      = 0;
    float m1N       = 0;
    float m1E       = 0;
    float m2N       = 0;
    float m2E       = 0;

    static std::string header()
    {
        return "targetLat, "
               "targetLon,targetN,targetE,EMCN,EMCE,M1N,M1E,M2N,M2E\n";
    }

    void print(std::ostream& os) const
    {
        os << targetLat << "," << targetLon << "," << targetN << "," << targetE
           << "," << emcN << "," << emcE << "," << m1N << "," << m1E << ","
           << m2N << "," << m2E << "\n";
    }
};

struct EarlyManeuversActiveTargetData
{
    uint64_t timestamp = 0;
    uint32_t target    = 0;  ///< Active target enumeration
    Boardcore::Units::Length::Meter altitude{
        0};  ///< Altitude when the target was changed

    static std::string header() { return "timestamp,target,altitude\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << target << "," << altitude << "\n";
    }
};

}  // namespace Parafoil
