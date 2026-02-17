/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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

#include "Valve.h"

#include "ValveServo.h"
#include "ValveServoPCA.h"
#include "ValveSolenoid.h"

namespace RIGv2
{
void Valve::unsafeSetServoPosition(float position)
{
    if (!servo)
        return;

    if (!(servo->getType() == ValveType::SOLENOID))
    {
        position *= config.limit;

        if (config.flipped)
            position = 1.0f - position;

        servo->setPosition(position);
    }
    else
    {
        servo->setPosition(position);
    }
}

float Valve::getServoPosition()
{
    if (!servo)
        return 0.0f;

    float position = servo->getPosition();

    if (servo->getType() == ValveType::TIMED)
    {
        if (config.flipped)
            position = 1.0f - position;

        position /= config.limit;
    }
    return position;
}

const Valve::ValveConfig Valve::getConfig() const { return Valve::config; }

}  // namespace RIGv2
