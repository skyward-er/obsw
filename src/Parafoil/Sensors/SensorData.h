/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <sensors/SensorData.h>

namespace Parafoil
{

struct SensorCalibrationData
{
    uint64_t timestamp = 0;
    float magBiasX     = 0.0f;
    float magBiasY     = 0.0f;
    float magBiasZ     = 0.0f;
    float magScaleX    = 0.0f;
    float magScaleY    = 0.0f;
    float magScaleZ    = 0.0f;

    static std::string header()
    {
        return "timestamp,magBiasX,magBiasY,magBiasZ,magScaleX,magScaleY,"
               "magScaleZ\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << magBiasX << "," << magBiasY << "," << magBiasZ
           << "," << magScaleX << "," << magScaleY << "," << magScaleZ << "\n";
    }
};

}  // namespace Parafoil