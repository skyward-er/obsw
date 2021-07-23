/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco
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

#include "Coeffs.h"

namespace DeathStackBoard
{
/**
 * @brief Pid proportional and integral controller with saturation
 * */
class Pid
{
private:
    const float Kp;
    const float Ki;

    float i         = 0;
    bool saturation = false;

public:
    Pid(float Kp, float Ki) : Kp(Kp), Ki(Ki) {}
    ~Pid() {}

    /**
     * @brief Update the Pid internal state
     * */
    float step(float umin, float umax, float error)
    {

        float p = Kp * error;

        if (!saturation)
        {
            i = i + Ki * error;
        }

        float u = p + i;

        if (u < umin)
        {
            u          = umin;
            saturation = true;
        }
        else if (u > umax)
        {
            u          = umax;
            saturation = true;
        }
        else
        {
            saturation = false;
        }

        return u;
    }
};

}  // namespace DeathStackBoard
