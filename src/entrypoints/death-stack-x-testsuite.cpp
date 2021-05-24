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

#include <Common.h>
#include <drivers/adc/ADS1118/ADS1118.h>
#include <drivers/adc/InternalADC/InternalADC.h>
#include <drivers/gps/ublox/UbloxGPS.h>
#include <drivers/hbridge/HBridge.h>
#include <drivers/spi/SPIDriver.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/MS580301BA07/MS580301BA07.h>
#include <sensors/analog/battery/BatteryVoltageSensor.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130A.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>

#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include "PinHandler/PinHandler.h"
#include "drivers/servo/servo.h"
#include "math/Stats.h"

using namespace std;

namespace PowerBoardTest
{
#include "../tests/deathstack-boards/test-power-board.cpp"
}

namespace STMBoardTest
{
#include "../tests/deathstack-boards/test-stm-board.cpp"
}

namespace RFBoardTest
{
#include "../tests/deathstack-boards/test-rf-board.cpp"
}

namespace AnalogBoardTest
{
#include "../tests/deathstack-boards/test-analog-board.cpp"
}

int menu();

int main()
{
    TimestampTimer::enableTimestampTimer();

    switch (menu())
    {
        case 1:
            PowerBoardTest::main();
            break;
        case 2:
            STMBoardTest::main();
            break;
        case 3:
            RFBoardTest::main();
            break;
        case 4:
            AnalogBoardTest::main();
            break;

        default:
            break;
    }
}

int menu()
{
    string temp;
    int choice;

    printf("\n\nWhat do you want to do?\n");
    printf("1. Test power board\n");
    printf("2. Test stm board\n");
    printf("3. Test rf board\n");
    printf("4. Test analog board\n");
    printf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}
