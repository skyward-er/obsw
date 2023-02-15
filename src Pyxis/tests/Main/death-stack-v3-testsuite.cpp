/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/GeneralPurposeTimer.h>
#include <drivers/timer/PWM.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/ADS131M04/ADS131M04.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/MS5803/MS5803.h>
#include <sensors/SensorSampler.h>
#include <stdint.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <array>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include "drivers/usart/USART.h"
#include "kernel/logging.h"

using namespace std;
using namespace miosix;

namespace STMBoard
{
#include "boards/test-stm-board.cpp"
}

namespace SensorBoard
{
#include "boards/test-sensors-board.cpp"
}

namespace PowerBoard
{
#include "boards/test-power-board.cpp"
}

namespace RFBoard
{
#include "boards/test-power-board.cpp"
}

// defining the interrupt for the test of the SensorBoard::BMX160
void __attribute__((used)) EXTI3_IRQHandlerImpl()
{
    using namespace SensorBoard;
    using namespace testBMX160;

    tick = TimestampTimer::getTimestamp();

    if (sensor)
        sensor->IRQupdateTimestamp(tick);
}

int menu();

int main()
{
    while (true)
    {
        switch (menu())
        {
            case 1:
                STMBoard::main();
                break;
            case 2:
                SensorBoard::main();
                break;
            case 3:
                PowerBoard::main();
                break;
            case 4:
                RFBoard::main();
                break;
            default:
                break;
        }
    }
}

int menu()
{
    string temp;
    int choice;

    iprintf("\n\nWhat do you want to do?\n");
    iprintf("1. Test STM board\n");
    iprintf("2. Test Sensors board\n");
    iprintf("3. Test power board\n");
    iprintf("4. Test RF board\n");
    iprintf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}
