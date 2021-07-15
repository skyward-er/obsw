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

#include <miosix.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "../../boards/DeathStack/AeroBrakesController/AeroBrakesServo.h"
#include "../../boards/DeathStack/DeploymentController/DeploymentServo.h"

using namespace std;
using namespace DeathStackBoard;

int servoMenu();
int actionMenu();
void waitUserInput();
void testServo();
void wiggleServo();
void setServoFullyOpen();
void setServoFullyClose();
void manualServoControl();
int askMinutes();
void servoBreakIn();

// Noise generating functions
int noise2(int x, int y);
float smooth_inter(float x, float y, float s);
float noise2d(float x, float y);
float perlin2d(float x, float y, float freq, int depth);

int servoChoice;
ServoInterface* servo;
PWMChannel channel;

constexpr int WIGGLE_STEPS = 5;

static int hashh[] = {
    208, 34,  231, 213, 32,  248, 233, 56,  161, 78,  24,  140, 71,  48,  140,
    254, 245, 255, 247, 247, 40,  185, 248, 251, 245, 28,  124, 204, 204, 76,
    36,  1,   107, 28,  234, 163, 202, 224, 245, 128, 167, 204, 9,   92,  217,
    54,  239, 174, 173, 102, 193, 189, 190, 121, 100, 108, 167, 44,  43,  77,
    180, 204, 8,   81,  70,  223, 11,  38,  24,  254, 210, 210, 177, 32,  81,
    195, 243, 125, 8,   169, 112, 32,  97,  53,  195, 13,  203, 9,   47,  104,
    125, 117, 114, 124, 165, 203, 181, 235, 193, 206, 70,  180, 174, 0,   167,
    181, 41,  164, 30,  116, 127, 198, 245, 146, 87,  224, 149, 206, 57,  4,
    192, 210, 65,  210, 129, 240, 178, 105, 228, 108, 245, 148, 140, 40,  35,
    195, 38,  58,  65,  207, 215, 253, 65,  85,  208, 76,  62,  3,   237, 55,
    89,  232, 50,  217, 64,  244, 157, 199, 121, 252, 90,  17,  212, 203, 149,
    152, 140, 187, 234, 177, 73,  174, 193, 100, 192, 143, 97,  53,  145, 135,
    19,  103, 13,  90,  135, 151, 199, 91,  239, 247, 33,  39,  145, 101, 120,
    99,  3,   186, 86,  99,  41,  237, 203, 111, 79,  220, 135, 158, 42,  30,
    154, 120, 67,  87,  167, 135, 176, 183, 191, 253, 115, 184, 21,  233, 58,
    129, 233, 142, 39,  128, 211, 118, 137, 139, 255, 114, 20,  218, 113, 154,
    27,  127, 246, 250, 1,   8,   198, 250, 209, 92,  222, 173, 21,  88,  102,
    219};

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
    printf("5.  Servo break-in (rodaggio)\n");
    printf("6.  Exit\n");
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
        servo   = new DeploymentServo{};
        channel = DPL_SERVO_PWM_CH;
    }
    else
    {
        servo   = new AeroBrakesServo{};
        channel = AB_SERVO_PWM_CH;
    }

    servo->enable();
    servo->reset();

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
                servoBreakIn();
                break;
            case 6:
                return;
                break;

            default:
                break;
        }
    }

    servo->reset();
    miosix::Thread::sleep(1000);
    servo->disable();
}

void wiggleServo()
{
    printf("\n\n** WIGGLE SERVO **\n\n");

    waitUserInput();

    printf("Wiggling ...\n");

    servo->selfTest();
    miosix::Thread::sleep(1000);
    servo->disable();

    printf("\n\tTest finished!\n\n");
}

void setServoFullyOpen()
{
    printf("\n\n** SERVO FULLY OPEN **\n\n");

    waitUserInput();

    servo->setMaxPosition();

    printf("\n\tTest finished!\n\n");
}

void setServoFullyClose()
{
    printf("\n\n** SERVO FULLY CLOSE **\n\n");

    waitUserInput();

    servo->setMinPosition();

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

    servo->set(angle);

    printf("\n\tTest finished!\n\n");
}

int askMinutes()
{
    int seconds;

    printf("How many minutes the test should run?\n");
    printf("\n>> ");
    scanf("%d", &seconds);

    return seconds;
}

void servoBreakIn()
{
    printf("\n\n** SERVO BREAK-IN **\n\n");

    uint64_t minutes = askMinutes();

    waitUserInput();

    uint64_t start = TimestampTimer::getTimestamp();

    while (TimestampTimer::getTimestamp() - start < minutes * 60 * 1000000)
    {
        uint64_t start2 = TimestampTimer::getTimestamp();
        float counter   = 0;

        // 10 seconds
        while (TimestampTimer::getTimestamp() - start2 < 10 * 1000000)
        {
            double angolo =
                servo->MIN_POS + abs(perlin2d(counter, 0, .075, 10)) *
                                     (servo->MAX_POS - servo->MIN_POS);
            servo->set(angolo);

            counter++;

            miosix::delayMs(10);
        }

        for (int i = 0; i < 10; i++)
        {
            servo->setMinPosition();
            miosix::delayMs(1500);
            servo->setMaxPosition();
            miosix::delayMs(1500);
        }
    }
}

int noise2(int x, int y)
{
    int tmp = hashh[y % 256];
    return hashh[(tmp + x) % 256];
}

float smooth_inter(float x, float y, float s)
{
    return x + (s * s * (3 - 2 * s)) * (y - x);
}

float noise2d(float x, float y)
{
    int x_int    = x;
    int y_int    = y;
    float x_frac = x - x_int;
    float y_frac = y - y_int;
    int s        = noise2(x_int, y_int);
    int t        = noise2(x_int + 1, y_int);
    int u        = noise2(x_int, y_int + 1);
    int v        = noise2(x_int + 1, y_int + 1);
    float low    = smooth_inter(s, t, x_frac);
    float high   = smooth_inter(u, v, x_frac);
    return smooth_inter(low, high, y_frac);
}

float perlin2d(float x, float y, float freq, int depth)
{
    float xa  = x * freq;
    float ya  = y * freq;
    float amp = 1.0;
    float fin = 0;
    float div = 0.0;

    int i;
    for (i = 0; i < depth; i++)
    {
        div += 256 * amp;
        fin += noise2d(xa, ya) * amp;
        amp /= 2;
        xa *= 2;
        ya *= 2;
    }

    return fin / div;
}