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

namespace Payload
{

struct WingTargetPositionData
{
    float receivedLat;
    float receivedLon;
    float targetN;
    float targetE;
    float emcN;
    float emcE;
    float m1N;
    float m1E;
    float m2N;
    float m2E;

    static std::string header()
    {
        return "receivedLat, receivedLon, "
               "targetN,targetE,EMCN,EMCE,M1N,M1E,M2N,M2E\n";
    }

    void print(std::ostream& os) const
    {
        os << receivedLat << "," << receivedLon << "," << targetN << ","
           << targetE << "," << emcN << "," << emcE << "," << m1N << "," << m1E
           << "," << m2N << "," << m2E << "\n";
    }
};

}  // namespace Payload
