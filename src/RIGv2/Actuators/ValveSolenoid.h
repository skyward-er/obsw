// Gpio come source (costruttore)
// Viene alzato a 1 o a 0 dopo setPosition.
/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include "ValveInterface.h"
#include "actuators/Servo/Servo.h"

namespace Boardcore
{
class ValveSolenoid : public RIGv2::ValveInterface
{
public:
    ValveSolenoid() {};  // TODO: Constructor with GPIO pin
    ~ValveSolenoid() {};

    bool setPosition(float position)
    {
        if (position < 0.5f)
            // set PIN to low
            return true;
        else
            // set PIN to high
            return false;
    };

    bool getPosition()
    {
        // read PIN state
        return false;  // TODO: return actual PIN state
    };
};
}  // namespace Boardcore
