/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>
#include <drivers/pwm/pwm.h>

#include <iostream>
#include <sstream>

#include "../../boards/DeathStack/configs/AeroBrakesConfig.h"
#include "../../boards/DeathStack/configs/DeploymentConfig.h"
#include "drivers/servo/servo.h"

using namespace std;
using namespace DeathStackBoard;
using namespace DeploymentConfigs;
using namespace AeroBrakesConfigs;

int servoMenu();
int actionMenu();
void waitUserInput();
void testServo();
void wiggleServo();
void setServoFullyOpen();
void setServoFullyClose();
void manualServoControl();

int servoChoice;
Servo *servo;
PWMChannel channel;

constexpr int WIGGLE_STEPS = 5;

int main()
{
    TimestampTimer::enableTimestampTimer();

    servoChoice = servoMenu();

    switch (servoChoice)
    {
        case 1:
            testServo();
            break;
        case 2:
            testServo();
            break;

        default:
            break;
    }

    return 0;
}

int servoMenu()
{
    string temp;
    int choice;

    printf("\nWhat do you want to move?\n");
    printf("1. Deployment servo\n");
    printf("2. Aerobrakes servo\n");
    printf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}

int actionMenu()
{
    string temp;
    int choice;

    printf("\nWhat do you want to do?\n");
    printf("1.  Servo wiggle (self test)\n");
    printf("2.  Servo fully open\n");
    printf("3.  Servo fully close\n");
    printf("4.  Servo manual control\n");
    printf("5.  Exit\n");
    printf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}

void waitUserInput()
{
    string temp;

    do
    {
        printf("Write 'start' to begin the test:\n");
        getline(cin, temp);
    } while (temp != "start");
}

void testServo()
{
    if (servoChoice == 1)
    {
        servo   = new Servo{DPL_SERVO_TIMER};
        channel = DPL_SERVO_PWM_CH;
    }
    else
    {
        servo   = new Servo{AB_SERVO_TIMER};
        channel = AB_SERVO_PWM_CH;
    }

    servo->setMinPulseWidth(500);
    servo->setMaxPulseWidth(2500);
    servo->setPosition(channel, 0);
    servo->enable(channel);
    servo->start();

    while (true)
    {
        switch (actionMenu())
        {
            case 1:
                wiggleServo();
                break;
            case 2:
                setServoFullyOpen();
                break;
            case 3:
                setServoFullyClose();
                break;
            case 4:
                manualServoControl();
                break;
            case 5:
                return;
                break;

            default:
                break;
        }
    }

    servo->stop();
    servo->disable(channel);
}

void wiggleServo()
{
    printf("\n\n** WIGGLE SERVO **\n\n");

    waitUserInput();

    printf("Wiggling ...\n");

    for (float step = 0; step <= 1; step += 1 / (float)WIGGLE_STEPS)
    {
        printf("step: %d°\n", (int)(step * 180));
        servo->setPosition(channel, step);
        miosix::Thread::sleep(1000);
    }

    printf("\n\tTest finished!\n\n");
}

void setServoFullyOpen()
{
    printf("\n\n** SERVO FULLY OPEN **\n\n");

    waitUserInput();

    servo->setPosition(channel, 1);

    printf("\n\tTest finished!\n\n");
}

void setServoFullyClose()
{
    printf("\n\n** SERVO FULLY CLOSE **\n\n");

    waitUserInput();

    servo->setPosition(channel, 0);

    printf("\n\tTest finished!\n\n");
}

void manualServoControl()
{
    printf("\n\n** MANUAL SERVO CONTROL **\n\n");

    string temp;
    float angle;

    do
    {
        printf("Write the servo postion in degrees:\n");
        getline(cin, temp);
        stringstream(temp) >> angle;
    } while (angle < 0 || angle > 180);

    servo->setPosition(channel, angle / 180);

    printf("\n\tTest finished!\n\n");
}
