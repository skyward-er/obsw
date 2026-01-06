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
#include "drivers/PCA9685/PCA9685.h"

namespace Boardcore
{
class ValveServoPCA : public RIGv2::ValveInterface
{
public:
    ValveServoPCA::ValveServoPCA(PCA9685& pca, PCA9685::Channels channel)
        : pca(pca), channel(channel)
    {
    }

    ///< Delete copy/move constructors/operators.
    ValveServoPCA(const ValveServoPCA&)            = delete;
    ValveServoPCA& operator=(const ValveServoPCA&) = delete;
    ValveServoPCA(ValveServoPCA&&)                 = delete;
    ValveServoPCA& operator=(ValveServoPCA&&)      = delete;

    bool setPosition(float position)
    {
        if (position < 0.0f)
            position = 0.0f;
        else if (position > 1.0f)
            position = 1.0f;

        return pca.setDutyCycle(channel, position);
    };

    bool getPosition() {};  // TODO: implement getPosition method

private:
    PCA9685& pca;
    PCA9685::Channels channel;
};
}  // namespace Boardcore
