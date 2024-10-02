/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <AutomatedAntennas/Actuators.h>
#include <AutomatedAntennas/Buses.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "actuators/stepper/StepperPWM.h"

using namespace miosix;
using namespace Boardcore;
using namespace Antennas;

/**
 * Test used to characterize the automated antennas.
 */

int main()
{
    bool initResult          = true;
    PrintLogger logger       = Logging::getLogger("automated_antennas");
    ModuleManager& modules   = ModuleManager::getInstance();
    TaskScheduler* scheduler = new TaskScheduler();

    // Starting singleton
    {
        scheduler->start();
#ifndef NO_SD_LOGGING
        printf("Starting Logger\n");
        if (!Logger::getInstance().start())
        {
            initResult = false;
            printf("Error initializing the Logger\n");
        }
        else
        {
            printf("Logger started successfully\n");
        }
#endif
    }

    // Initialize the modules
    {
        if (!modules.insert<Buses>(new Buses()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Buses module");
        }

        if (!modules.insert<Actuators>(new Actuators()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Actuators module");
        }
    }

    modules.get<Actuators>()->start();

    if (!initResult)
    {
        printf("Errors present during module insertion\n");
        return -1;
    }

    // Periodically statistics
    for (auto i = 0; i < 50; i++)
    {
        modules.get<Actuators>()->moveDeg(Actuators::StepperList::VERTICAL, 90);
        Thread::sleep(100);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        StackLogger::getInstance().log();
    }

    for (auto i = 0; i < 50; i++)
    {
        modules.get<Actuators>()->moveDeg(Actuators::StepperList::HORIZONTAL,
                                          90);
        Thread::sleep(100);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        StackLogger::getInstance().log();
    }

    printf("Logging finished\n");
    Logger::getInstance().stop();
    printf("Logging stopped\n");

    while (1)
        ;
}
