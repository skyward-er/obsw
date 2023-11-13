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

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/Buses.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ButtonHandler/ButtonHandler.h>

#include <iostream>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "actuators/stepper/StepperPWM.h"

using namespace miosix;
using namespace Boardcore;
using namespace Antennas;

/**
 * Test used to characterize the automated antennas.
 */

GpioPin button = GpioPin(GPIOC_BASE, 13);

void __attribute__((used)) EXTI13_IRQHandlerImpl()
{
    ModuleManager::getInstance().get<Actuators>()->IRQemergencyStop();
}

int main()
{
    bool initResult          = true;
    PrintLogger logger       = Logging::getLogger("automated_antennas");
    ModuleManager& modules   = ModuleManager::getInstance();
    TaskScheduler* scheduler = new TaskScheduler();

    button.mode(Mode::INPUT);
    enableExternalInterrupt(button.getPort(), button.getNumber(),
                            InterruptTrigger::RISING_EDGE);

    // INSERTING MODULES
    {
        if (!modules.insert<Buses>(new Buses()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Buses module");
        }
        else
        {
            LOG_INFO(logger, "Successfully inserted Buses module\n");
        }

        if (!modules.insert<Actuators>(new Actuators()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Actuators module");
        }
        else
        {
            LOG_INFO(logger, "Successfully inserted Actuators module\n");
        }

        if (!modules.insert<Sensors>(new Sensors()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Sensors module");
        }
        else
        {
            LOG_INFO(logger, "Successfully inserted Sensors module\n");
        }

        // Insert algorithm modules
    }

    // ADDING TASKS
    {
        scheduler->addTask(
            []()
            {
                Logger::getInstance().log(CpuMeter::getCpuStats());
                CpuMeter::resetCpuStats();
                Logger::getInstance().logStats();
                StackLogger::getInstance().log();
            },
            100);
    }

    // STARTING MODULES
    {
#ifndef NO_SD_LOGGING
        // Starting the Logger
        if (!Logger::getInstance().start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing the Logger\n");
        }
        else
        {
            LOG_INFO(logger, "Logger started successfully\n");
        }
#endif

        // Starting scheduler
        if (!scheduler->start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing the Scheduler\n");
        }
        else
        {
            LOG_INFO(logger, "Scheduler started successfully\n");
        }

        // Starting the Actuators
        modules.get<Actuators>()->start();

        // Starting the Sensors
        if (!modules.get<Sensors>()->start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing the Sensors\n");
        }
        else
        {
            LOG_INFO(logger, "Sensors started successfully\n");
        }
    }

    if (!initResult)
    {
        printf("Errors present during module insertion\n");
        return -1;
    }

    // Periodically statistics
    float speedMax  = 1.0;
    float speed0    = 0.5;
    float speed     = speed0;
    float stepSpeed = 0.1;
    bool increasing = true;

    modules.get<Actuators>()->setSpeed(Actuators::StepperList::HORIZONTAL,
                                       speed);

    modules.get<Actuators>()->setSpeed(Actuators::StepperList::VERTICAL, speed);
    // scheduler->addTask(
    //     [&]()
    //     {
    //         if (increasing)
    //         {
    //             if (speed < speedMax)
    //             {
    //                 speed += stepSpeed;
    //                 modules.get<Actuators>()->setSpeed(
    //                     Actuators::StepperList::HORIZONTAL, speed);
    //             }
    //             else
    //             {
    //                 increasing = false;
    //             }
    //         }
    //         else
    //         {
    //             if (speed > 0)
    //             {
    //                 speed -= stepSpeed;
    //                 modules.get<Actuators>()->setSpeed(
    //                     Actuators::StepperList::HORIZONTAL, speed);
    //             }
    //             else
    //             {
    //                 increasing = true;
    //             }
    //         }
    //     },
    //     100);
    VN300Data data;
    while (!ModuleManager::getInstance().get<Actuators>()->isEmergencyStopped())
    {
        {
            data = modules.get<Sensors>()->getVN300LastSample();
            printf("acc[%.3f,%.3f,%.3f] ypr[%.3f,%.3f,%.3f]\n",
                   data.accelerationX, data.accelerationY, data.accelerationZ,
                   data.yaw, data.pitch, data.roll);
            modules.get<Actuators>()->moveDeg(
                Actuators::StepperList::HORIZONTAL, 90);
            modules.get<Actuators>()->moveDeg(Actuators::StepperList::VERTICAL,
                                              90);
            Thread::sleep(1500);
        }

        {
            data = modules.get<Sensors>()->getVN300LastSample();
            printf("acc[%.3f,%.3f,%.3f] ypr[%.3f,%.3f,%.3f]\n",
                   data.accelerationX, data.accelerationY, data.accelerationZ,
                   data.yaw, data.pitch, data.roll);
            modules.get<Actuators>()->moveDeg(
                Actuators::StepperList::HORIZONTAL, -90);
            modules.get<Actuators>()->moveDeg(Actuators::StepperList::VERTICAL,
                                              -90);
            Thread::sleep(1500);
        }
    }

    // Stopping threads
    {
        Logger::getInstance().stop();
        printf("Logger stopped! Board can be reset/shutdown9\n");
    }

    while (1)
        ;
}
