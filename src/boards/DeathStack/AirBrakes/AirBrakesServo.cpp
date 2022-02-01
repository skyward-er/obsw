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

#include <AirBrakes/AirBrakesServo.h>

using namespace DeathStackBoard::AirBrakesConfigs;

namespace DeathStackBoard
{

AirBrakesServo::AirBrakesServo(float minPosition, float maxPosition,
                               float resetPosition)
    : ServoInterface(minPosition, maxPosition, resetPosition),
      servo(AB_SERVO_TIMER, Boardcore::TimerUtils::Channel::CHANNEL_2, 500,
            2500, 50)
{
}

AirBrakesServo::~AirBrakesServo() {}

void AirBrakesServo::enable() { servo.enable(); }

void AirBrakesServo::disable() { servo.disable(); }

void AirBrakesServo::selfTest()
{
    float base   = (MAX_POS + RESET_POS) / 2;
    float maxpos = base + AB_SERVO_WIGGLE_AMPLITUDE / 2;
    float minpos = base - AB_SERVO_WIGGLE_AMPLITUDE / 2;

    set(base, true);

    for (int i = 0; i < 3; i++)
    {
        miosix::Thread::sleep(ABK_UPDATE_PERIOD + 100);
        set(maxpos, true);
        miosix::Thread::sleep(ABK_UPDATE_PERIOD + 100);
        set(minpos, true);
    }

    miosix::Thread::sleep(ABK_UPDATE_PERIOD);
    reset();
}

void AirBrakesServo::setPosition(float angle)
{
    currentPosition = angle;
    servo.setPosition180Deg(angle);

#ifdef HARDWARE_IN_THE_LOOP
    simulator->send(angle);
#endif
}

float AirBrakesServo::preprocessPosition(float angle)
{
    angle = ServoInterface::preprocessPosition(angle);

    float rate = (angle - currentPosition) / ABK_UPDATE_PERIOD_SECONDS;

    if (rate > AB_SERVO_MAX_RATE)
    {
        angle = ABK_UPDATE_PERIOD_SECONDS * AB_SERVO_MAX_RATE + currentPosition;
    }
    else if (rate < AB_SERVO_MIN_RATE)
    {
        angle = ABK_UPDATE_PERIOD_SECONDS * AB_SERVO_MIN_RATE + currentPosition;
    }

    angle = FILTER_COEFF * angle + (1 - FILTER_COEFF) * getCurrentPosition();

    return angle;
}

}  // namespace DeathStackBoard
