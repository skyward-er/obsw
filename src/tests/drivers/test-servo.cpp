/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <arch/common/drivers/servo_stm32.h>
#include <miosix.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "drivers/servo/servo.h"
#include "configs/DeploymentConfig.h"

using namespace miosix;
using namespace std;
using namespace DeathStackBoard;

static const PWMChannel CHANNEL = PWMChannel::CH1;

int main()
{
    Servo s{DeploymentConfigs::DPL_SERVO_TIMER};
    s.setMinPulseWidth(500);
    s.setMaxPulseWidth(2500);
    s.setPosition(CHANNEL, 0.77f);
    s.enable(CHANNEL);

    s.start();
    // Thread::sleep(5000);
    string temp;
    printf("Ready.\n");
    int pos;
    for (;;)
    {
        temp = "";
        printf("Insert position: \n");
        getline(cin, temp);

        stringstream(temp) >> pos;

        if (pos >= 0 && pos <= 100)
        {
            printf("Position set to %d\n", pos);
            s.setPosition(CHANNEL, pos / 100.0f);
        }
        else
        {
            printf("You dumb fuck.\n");
        }
        Thread::sleep(100);
    }
}