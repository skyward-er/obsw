/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <Wing/WingServo.h>

namespace ParafoilTestDev
{
    /**
     * PUBLIC METHODS
     */
    WingServo::WingServo(TIM_TypeDef* timer, TimerUtils::Channel pwmChannel, float minPosition, float maxPosition)
             :WingServo(timer, pwmChannel, minPosition, maxPosition, minPosition){}
    
    WingServo::WingServo(TIM_TypeDef* timer, TimerUtils::Channel pwmChannel, float minPosition, float maxPosition, float resetPosition)
             :ServoInterface(minPosition, maxPosition, resetPosition)
    {
        servo = new Servo(timer, pwmChannel);
    }

    WingServo::~WingServo(){}

    void WingServo::enable()
    {
        servo -> enable();
    }

    void WingServo::disable()
    {
        servo -> disable();
    }

    void WingServo::selfTest()
    {
        setPosition(MIN_POS);
        miosix::Thread::sleep(1000);
        setPosition(RESET_POS);
        miosix::Thread::sleep(1000);
        setPosition(MAX_POS);
    }

    /**
     * PRIVATE METHODS
     */
    void WingServo::setPosition(float angle)
    {
        servo->setPosition(angle / 180);
    }

    float WingServo::preprocessPosition(float angle)
    {
        return angle;
    }
}