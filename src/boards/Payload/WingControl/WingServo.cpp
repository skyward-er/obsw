/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <Payload/WingControl/WingServo.h>

using namespace Boardcore;
using namespace PayloadBoard::WingConfigs;

namespace PayloadBoard
{

WingServo::WingServo(TIM_TypeDef* const timer, TimerUtils::Channel channel,
                     float minPosition, float maxPosition)
    : ServoInterface(minPosition, maxPosition),
      servo(timer, channel, 50, 500, 2500)
{
}

WingServo::~WingServo() {}

void WingServo::enable() { servo.enable(); }

void WingServo::disable() { servo.disable(); }

void WingServo::selfTest()
{
    float base   = (MAX_POS + RESET_POS) / 2;
    float maxpos = base + WING_SERVO_WIGGLE_AMPLITUDE / 2;
    float minpos = base - WING_SERVO_WIGGLE_AMPLITUDE / 2;

    set(base, true);

    for (int i = 0; i < 3; i++)
    {
        miosix::Thread::sleep(500);
        set(maxpos, true);
        miosix::Thread::sleep(500);
        set(minpos, true);
    }

    miosix::Thread::sleep(500);
    reset();
}

void WingServo::setPosition(float angle)
{
    this->currentPosition = angle;
    // map position to [0;1] interval for the servo driver
    servo.setPosition180Deg(angle);
}

}  // namespace PayloadBoard
