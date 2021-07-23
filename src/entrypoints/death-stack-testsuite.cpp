/* Copyright (c) 2019-2020 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#include <TMTCManager/TMTCManager.h>
#include <diagnostic/CpuMeter.h>
#include <drivers/BusTemplate.h>
#include <drivers/piksi/piksi.h>
#include <interfaces-impl/hwmapping.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <sensors/ADIS16405/ADIS16405.h>
#include <sensors/LM75B.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorSampling.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "ADA/ADA.h"
#include "Common.h"
#include "DeploymentController/Motor/MotorDriver.h"
#include "DeploymentController/ThermalCutter/Cutter.h"
#include "LoggerService/LoggerService.h"
#include "SensorManager/SensorManager.h"
#include "SensorManager/Sensors/AD7994Wrapper.h"
#include "SensorManager/Sensors/ADCWrapper.h"
#include "configs/SensorManagerConfig.h"
#include "diagnostic/CpuMeter.h"
#include "drivers/HardwareTimer.h"
#include "drivers/Xbee/Xbee.h"
#include "events/EventBroker.h"
#include "math/Stats.h"
#include "skyward-boardcore/src/tests/logger/test-logger.h"

using namespace std;
using namespace miosix;

using namespace DeathStackBoard;

using namespace std;
using namespace miosix;

namespace sensortest
{
#include "../tests/drivers/test-all-sensors.cpp"
}

namespace thermotest
{
#include "../tests/drivers/test-cutter.cpp"
}

namespace motortest
{
#include "../tests/drivers/test-motor.cpp"
}

namespace loggertest
{
#include "skyward-boardcore/src/tests/logger/test-logger.cpp"
}

namespace xbeetest
{
#include "skyward-boardcore/src/tests/misc/xbee-send-rcv.cpp"
}

namespace sm_tmtc
{
#include "../tests/test-sm+tmtc.cpp"
}

void banner()
{
    printf(" _____             _   _        _____ _             _    \n");
    printf("|  __ \\           | | | |      / ____| |           | |   \n");
    printf("| |  | | ___  __ _| |_| |__   | (___ | |_ __ _  ___| | __\n");
    printf("| |  | |/ _ \\/ _` | __| '_ \\   \\___ \\| __/ _` |/ __| |/ /\n");
    printf("| |__| |  __/ (_| | |_| | | |  ____) | || (_| | (__|   < \n");
    printf(
        "|_____/ \\___|\\__,_|\\__|_| |_| |_____/ "
        "\\__\\__,_|\\___|_|\\_\\\n\n");
}

int main()
{
    banner();

    while (true)
    {
        printf("Choose a test:\n");
        printf(" s - Test All Sensors\n");
        printf(" t - Thermal Cutter test\n");
        printf(" m - Nosecone Motor\n");
        printf(" g - Sensors + TMTC (telemetry/telecommands)\n");
        printf(" x - XBee send/rcv\n");
        printf(" l - Logger\n");
        printf("\nOther:\n");
        printf(" r - Reboot\n");
        printf(" f - Pay Respect\n");

        // Do not directly use cin -- use getline
        char c;
        string temp;
        getline(cin, temp);
        stringstream(temp) >> c;

        switch (c)
        {
            case 's':
            {
                sensortest::main();
                break;
            }
            case 't':
            {
                thermotest::main();
                break;
            }
            case 'm':
            {
                motortest::main();
                break;
            }
            case 'x':
            {
                xbeetest::main();
                break;
            }
            case 'l':
            {
                loggertest::main();
                break;
            }
            case 'g':
            {
                sm_tmtc::main();
                break;
            }
            case 'r':
            {
                printf("Rebooting\n");
                miosix::reboot();
                break;
            }
            case 'f':
            {
                printf("....................../´¯/) \n");
                printf("....................,/¯../ \n");
                printf(".................../..../ \n");
                printf("............./´¯/'...'/´¯¯`·¸ \n");
                printf("........../'/.../..../......./¨¯\\ \n");
                printf("........('(...´...´.... ¯~/'...') \n");
                printf(".........\\.................'...../ \n");
                printf("..........''...\\.......... _.·´ \n");
                printf("............\\..............( \n");
                printf("..............\\.............\\...\n\n");
                break;
            }
            default:
                break;
        }
    }
}