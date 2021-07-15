/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

/**
 * Components on the stm board:
 *
 * STM32F429:
 *     Led wave
 *     External oscillator
 */

#include <Common.h>

#include <iostream>
#include <sstream>

#include "math/Stats.h"

using namespace std;

namespace LedWave
{
#include "../ledwave.cpp"
}

namespace HSE
{
#include "../test-hse.cpp"
}

namespace SDCardBenchmark
{
#include "../../skyward-boardcore/src/entrypoints/sdcard-benchmark.cpp"
}

// Sample frequency
constexpr int SAMPLING_FREQUENCY = 100;

int menu();
int askSeconds();

int main()
{
    TimestampTimer::enableTimestampTimer();

    switch (menu())
    {
        case 1:
            LedWave::main();
            break;
        case 2:
            HSE::main();
            break;
        case 3:
            SDCardBenchmark::main();
            break;

        default:
            break;
    }

    return 0;
}

int menu()
{
    string temp;
    int choice;

    printf("\n\nWhat do you want to do?\n");
    printf("1. Ledwave\n");
    printf("2. External oscillator\n");
    printf("3. SD card benchmark\n");
    printf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}

int askSeconds()
{
    int seconds;

    printf("How many seconds the test should run?\n");
    printf("\n>> ");
    scanf("%d", &seconds);

    return seconds;
}
