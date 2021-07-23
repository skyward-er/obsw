/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <interfaces-impl/hwmapping.h>

#include <iostream>
#include <sstream>
#include <string>

#include "DeploymentController/Motor/MotorDriver.h"

using namespace miosix;
using namespace DeathStackBoard;
using namespace std;

enum motorState
{
    STOPPED,
    NORMAL,
    REVERSE
};

MotorDriver motor;
motorState state = STOPPED;

void buttonMode()
{
    while (true)
    {
        if (inputs::btn_open::value() == 0)
        {
            Thread::sleep(10);

            motor.start(MotorDirection::NORMAL);

            while (!inputs::btn_open::value())
                ;

            motor.stop();
        }

        if (inputs::btn_close::value() == 0)
        {
            Thread::sleep(10);

            motor.start(MotorDirection::REVERSE);

            while (!inputs::btn_close::value())
                ;

            motor.stop();
        }
    }
}

int main()
{

    Thread::sleep(500);

    while (true)
    {
        printf("\nOptions:\n");
        printf(" o - open\n");
        printf(" c - close\n");
        printf(" s - stop\n");
        printf(" b - use buttons\n");
        printf(" q - quit\n");

        // Do not directly use cin -- use getline
        char c;
        string temp;
        getline(cin, temp);
        stringstream(temp) >> c;

        switch (c)
        {
            case 'o':
                motor.start(MotorDirection::NORMAL);
                break;
            case 'c':
                motor.start(MotorDirection::REVERSE);
                break;
            case 's':
                motor.stop();
                break;
            case 'b':
                buttonMode();
                break;
            case 'q':
                return 0;
                break;
            default:
                break;
        }
    }

    return 0;
}