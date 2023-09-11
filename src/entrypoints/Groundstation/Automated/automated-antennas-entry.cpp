/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Emilio Corigliano
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
#include <Groundstation/Automated/Follower/Follower.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Radio/Radio.h>
#include <Groundstation/Automated/Radio/RadioStatus.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/Ports/Serial.h>
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
#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "actuators/stepper/StepperPWM.h"

using namespace Groundstation;
using namespace Antennas;
using namespace Boardcore;
using namespace miosix;

GpioPin button = GpioPin(GPIOG_BASE, 10);  ///< Emergency stop button

void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    ModuleManager::getInstance().get<Actuators>()->IRQemergencyStop();
}

int main()
{
    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("automated_antennas");
    bool ok                = true;

    button.mode(Mode::INPUT);
    enableExternalInterrupt(button.getPort(), button.getNumber(),
                            InterruptTrigger::RISING_EDGE);

    TaskScheduler *scheduler  = new TaskScheduler();
    Hub *hub                  = new Hub();
    Buses *buses              = new Buses();
    Serial *serial            = new Serial();
    RadioMain *radio_main     = new RadioMain();
    RadioStatus *radio_status = new RadioStatus();
    Actuators *actuators      = new Actuators();
    Sensors *sensors          = new Sensors();
    Follower *follower        = new Follower();

    // Inserting Modules
    {
        ok &= modules.insert(follower);
        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(buses);
        ok &= modules.insert(serial);
        ok &= modules.insert(radio_main);
        ok &= modules.insert(radio_status);
        ok &= modules.insert(actuators);
        ok &= modules.insert(sensors);
    }

    // Starting Modules
    {
#ifndef NO_SD_LOGGING
        // Starting the Logger
        if (!Logger::getInstance().start())
        {
            ok = false;
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
            ok = false;
            LOG_ERR(logger, "Error initializing the Scheduler\n");
        }

        if (!serial->start())
        {
            ok = false;
            LOG_ERR(logger, "Failed to start serial!\n");
        }

        if (!radio_main->start())
        {
            ok = false;
            LOG_ERR(logger, "Failed to start main radio!\n");
        }

        if (!radio_status->start())
        {
            ok = false;
            LOG_ERR(logger, "Failed to start radio status!\n");
        }

        if (!sensors->start())
        {
            ok = false;
            LOG_ERR(logger, "Failed to start sensors!\n");
        }

        actuators->start();

        if (!follower->init())
        {
            ok = false;
            LOG_ERR(logger, "Failed to start follower!\n");
        }
    }

    follower->begin();
    scheduler->addTask(std::bind(&Follower::update, follower), 100);

    std::thread gpsFix(
        [&]()
        {
            GPSData antennaPosition;
            while (1)
            {
                VN300Data vn300Data = ModuleManager::getInstance()
                                          .get<Antennas::Sensors>()
                                          ->getVN300LastSample();

                if (vn300Data.fix_gps != 0)
                {
                    antennaPosition.gpsTimestamp  = vn300Data.insTimestamp;
                    antennaPosition.latitude      = vn300Data.latitude;
                    antennaPosition.longitude     = vn300Data.longitude;
                    antennaPosition.height        = vn300Data.altitude;
                    antennaPosition.velocityNorth = vn300Data.nedVelX;
                    antennaPosition.velocityEast  = vn300Data.nedVelY;
                    antennaPosition.velocityDown  = vn300Data.nedVelZ;
                    antennaPosition.satellites    = vn300Data.fix_gps;
                    antennaPosition.fix           = (vn300Data.fix_gps > 0);
                    LOG_INFO(logger,
                             "GPS fix "
                             "acquired !coord[%f, %f] ",
                             antennaPosition.latitude,
                             antennaPosition.longitude);
                    follower->setAntennaPosition(antennaPosition);

                    led3On();
                    return;
                }

                Thread::sleep(1000);
            }
        });

    if (radio_status->isMainRadioPresent())
    {
        led1On();
    }
    else
    {
        led2On();
    }

    gpsFix.join();

    while (true)
    {
        Thread::wait();
    }

    return 0;
}
