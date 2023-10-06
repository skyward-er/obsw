/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Federico Mandelli
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

namespace Payload
{

struct RadioSetterParameter
{
    uint64_t timestamp;
    float refAltitude;
    float refTemperature;
    float depAltitude;
    float yawOri;
    float pitchOri;
    float rollOri;
    float latCord;
    float lonCord;
    float latTargetCord;
    float lonTargetCord;
    uint8_t algoId;

    RadioSetterParameter()
        : timestamp(0), refAltitude(0), refTemperature(0), depAltitude(0),
          yawOri(0), pitchOri(0), rollOri(0), latCord(0), lonCord(0),
          latTargetCord(0), lonTargetCord(0), algoId(0)
    {
    }

    static std::string header()
    {
        return "timestamp,refAltitude,refTemperature,depAltitude,"
               "yawOrientation,pitchOrientation,rollOrientation,latCoordinates,"
               "lonCoordinates,latTargetCoordinates,lonTargetCoordinates,"
               "algoId\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << refAltitude << "," << refTemperature << ","
           << depAltitude << "," << yawOri << "," << pitchOri << "," << rollOri
           << "," << latCord << "," << lonCord << "," << latTargetCord << ","
           << lonTargetCord << "," << static_cast<int>(algoId) << "\n";
    }
};

}  // namespace Payload
