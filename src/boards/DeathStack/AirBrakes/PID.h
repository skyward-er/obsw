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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

namespace DeathStackBoard
{

/**
 * @brief Proportional and integral controller with saturation.
 * */
class PIController
{

public:
    PIController(float Kp, float Ki) : Kp(Kp), Ki(Ki) {}
    ~PIController() {}

    /**
     * @brief Update the PI internal state.
     * */
    float update(float error)
    {

        float p = Kp * error;

        if (!saturation)
        {
            i = i + Ki * error;
        }

        float u = p + i;

        return u;
    }

    /**
     * @brief Anti-windup mechanism.
     * */
    float antiWindUp(float u, float umin, float umax)
    {
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

private:
    const float Kp;       // proportional factor
    const float Ki;       // integral factor
    float i         = 0;  // integral contribution
    bool saturation = false;
};

}  // namespace DeathStackBoard
