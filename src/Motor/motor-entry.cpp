/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Motor/Actuators/Actuators.h>
#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/HIL/HIL.h>
#include <Motor/PersistentVars/PersistentVars.h>
#include <Motor/Sensors/HILSensors.h>
#include <Motor/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <iomanip>
#include <iostream>

using namespace Boardcore;
using namespace Motor;
using namespace miosix;

int main()
{
    ledOff();

    bool initResult = true;

    DependencyManager manager;

    Buses *buses              = new Buses();
    BoardScheduler *scheduler = new BoardScheduler();

    Sensors *sensors =
        (PersistentVars::getHilMode() ? new HILSensors(Config::HIL::ENABLE_HW)
                                      : new Sensors());
    Actuators *actuators   = new Actuators();
    CanHandler *canHandler = new CanHandler();

    Logger &sdLogger = Logger::getInstance();

    // HIL
    MotorHIL *hil = nullptr;
    if (PersistentVars::getHilMode())
    {
        hil = new MotorHIL();

        initResult = initResult && manager.insert(hil);
    }

    initResult = initResult && manager.insert<Buses>(buses) &&
                 manager.insert<BoardScheduler>(scheduler) &&
                 manager.insert<Sensors>(sensors) &&
                 manager.insert<Actuators>(actuators) &&
                 manager.insert<CanHandler>(canHandler) && manager.inject();

    manager.graphviz(std::cout);

    if (!initResult)
    {
        std::cout << "Failed to inject dependencies" << std::endl;
        return -1;
    }

    // Status led indicators
    // led1: Sensors ok
    // led2: Actuators ok
    // led3: CanBus ok
    // led4: Everything ok

    // Start modules
    std::cout << "Starting Actuators" << std::endl;
    if (!actuators->start())
    {
        initResult = false;
        std::cout << "*** Failed to start Actuators ***" << std::endl;
    }
    else
    {
        led2On();
    }

    std::cout << "Starting CanHandler" << std::endl;
    if (!canHandler->start())
    {
        initResult = false;
        std::cout << "*** Failed to start CanHandler ***" << std::endl;
    }
    else
    {
        led3On();
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cout << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    // Start logging when system boots
    std::cout << "Starting Logger" << std::endl;
    if (!sdLogger.start())
    {
        initResult = false;
        std::cout << "*** Failed to start Logger ***" << std::endl;
    }
    else
    {
        sdLogger.resetStats();
        std::cout << "Logger Ok!\n"
                  << "\tLog number: " << sdLogger.getStats().logNumber
                  << std::endl;
    }

    if (PersistentVars::getHilMode())
    {
        if (!hil->start())
        {
            initResult = false;
            std::cout << "Error failed to start HIL" << std::endl;
        }

        // Waiting for start of simulation
        hil->waitStartSimulation();
    }

    std::cout << "Starting Sensors" << std::endl;
    if (!sensors->start())
    {
        initResult = false;
        std::cout << "*** Failed to start Sensors ***" << std::endl;
    }
    else
    {
        led1On();
    }

    if (initResult)
    {
        canHandler->setInitStatus(InitStatus::INIT_OK);
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        canHandler->setInitStatus(InitStatus::INIT_ERR);
        std::cout << "*** Init failure ***" << std::endl;
    }

    std::cout << "Sensor status:" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        std::cout << "\t" << std::setw(16) << std::left << info.id << " "
                  << (info.isInitialized ? "Ok" : "Error") << std::endl;
    }

    std::cout << "Battery voltage: "
              << sensors->getBatteryVoltageLastSample().voltage << std::endl;

    while (true)
    {
        // Log CpuMeter
        sdLogger.log(sdLogger.getStats());
        sdLogger.log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();

        gpios::boardLed::high();
        Thread::sleep(1000);

        // Log CpuMeter
        sdLogger.log(sdLogger.getStats());
        sdLogger.log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();

        gpios::boardLed::low();
        Thread::sleep(1000);
    }

    return 0;
}