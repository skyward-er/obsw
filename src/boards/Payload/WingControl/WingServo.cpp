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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Payload/WingControl/WingServo.h>

namespace PayloadBoard
{

using namespace WingConfigs;

WingServo::WingServo(PWM::Timer servo_timer, PWMChannel servo_ch)
    : ServoInterface(WING_SERVO_MIN_POS, WING_SERVO_MAX_POS),
      servo(servo_timer), servo_channel(servo_ch)
{
}

WingServo::WingServo(PWM::Timer servo_timer, PWMChannel servo_ch,
                     float minPosition, float maxPosition)
    : ServoInterface(minPosition, maxPosition), servo(servo_timer),
      servo_channel(servo_ch)
{
}

WingServo::~WingServo() {}

void WingServo::enable()
{
    servo.setMaxPulseWidth(2500);
    servo.setMinPulseWidth(500);
    servo.enable(servo_channel);
    servo.start();
}

void WingServo::disable()
{
    servo.stop();
    servo.disable(servo_channel);
}

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
    servo.setPosition(servo_channel, angle / 180.0f);
}

}  // namespace PayloadBoard