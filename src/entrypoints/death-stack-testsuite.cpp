#include "DeathStack/configs/SensorManagerConfig.h"

#include "DeathStack/SensorManager/Sensors/AD7994Wrapper.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapper.h"

#include <drivers/piksi/piksi.h>
#include <sensors/ADIS16405/ADIS16405.h>
#include <sensors/LM75B.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorSampling.h>

#include <interfaces-impl/hwmapping.h>

#include <miosix.h>
#include <cstdio>
#include <iostream>
#include <string>

#include "drivers/HardwareTimer.h"
#include "drivers/Xbee/Xbee.h"
#include "math/Stats.h"

#include <drivers/BusTemplate.h>

#include <miosix.h>
#include "DeathStack/DeploymentController/ThermalCutter/Cutter.h"
#include <interfaces-impl/hwmapping.h>
#include "DeathStack/SensorManager/Sensors/ADCWrapper.h"
#include <iostream>

#include "DeathStack/DeploymentController/Motor/MotorDriver.h"

#include <cstdio>
#include <logger/Logger.h>
#include <diagnostic/CpuMeter.h>
#include "skyward-boardcore/src/tests/logger/test-logger.h"

#include "Common.h"
#include "DeathStack/ADA/ADA.h"
#include "DeathStack/LogProxy/LogProxy.h"
#include "DeathStack/SensorManager/SensorManager.h"
#include "diagnostic/CpuMeter.h"
#include "events/EventBroker.h"

#include <boards/DeathStack/TMTCManager/TMTCManager.h>

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
 printf("|_____/ \\___|\\__,_|\\__|_| |_| |_____/ \\__\\__,_|\\___|_|\\_\\\n\n");
}

int main()
{
    banner();

    while(true)
    {
        printf("Choose a test:\n");
        printf(" s - sensors\n");
        printf(" t - thermocutters\n");
        printf(" m - nosecone motor\n");
        printf(" g - gs (sm+tmtc)\n");
        printf(" x - xbee send/rcv\n");
        printf(" l - logger\n");
        printf(" r - reboot\n");
        printf(" f - pay respect\n");

        char c;
        scanf("%c", &c);

        while(getchar() != '\n');

        switch(c)
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