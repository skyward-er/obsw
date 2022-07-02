/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include <actuators/Servo/Servo.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ClockUtils.h>

#include <iostream>

using namespace Boardcore;
using namespace miosix;

Servo expulsion(TIM4, TimerUtils::Channel::CHANNEL_2);
Servo airbrakes(TIM10, TimerUtils::Channel::CHANNEL_1);
Servo auxiliary(TIM11, TimerUtils::Channel::CHANNEL_1);

// Position to cycle through for the servo 1, 2 and 3
float positions[] = {0, 0.5, 1.0};
int lastPosition  = 0;

void moveServo()
{
    expulsion.setPosition(positions[lastPosition % 3]);
    airbrakes.setPosition(positions[(lastPosition + 1) % 3]);
    auxiliary.setPosition(positions[(lastPosition + 2) % 3]);

    lastPosition++;
}

int main()
{

    // Enable the timers
    expulsion.enable();
    airbrakes.enable();
    auxiliary.enable();

    // Start a periodic task to move the first three servos
    TaskScheduler scheduler;
    scheduler.addTask(&moveServo, 2 * 1000, 1);
    scheduler.start();

    while (true)
        Thread::sleep(1000);
}
