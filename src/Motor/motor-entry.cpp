/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace Boardcore;
using namespace Motor;
using namespace miosix;

int main()
{
    ledOff();

    bool initResult = true;

    DependencyManager manager;

    Buses* buses              = new Buses();
    BoardScheduler* scheduler = new BoardScheduler();

    Sensors* sensors = nullptr;
    auto actuators   = new Actuators();
    auto canHandler  = new CanHandler();

    auto& sdLogger = Logger::getInstance();

    // HIL
    MotorHIL* hil = nullptr;
    if (PersistentVars::getHilMode())
    {
        hil = new MotorHIL();
        initResult &= manager.insert<MotorHIL>(hil);
        sensors = new HILSensors(Config::HIL::ENABLE_HW);
    }
    else
    {
        sensors = new Sensors();
    }

    initResult &= manager.insert<Buses>(buses) &&
                  manager.insert<BoardScheduler>(scheduler) &&
                  manager.insert<Sensors>(sensors) &&
                  manager.insert<Actuators>(actuators) &&
                  manager.insert<CanHandler>(canHandler) && manager.inject();

    if (!initResult)
    {
        std::cerr << "*** Failed to inject dependencies ***" << std::endl;
        return -1;
    }

    // Status led indicators
    // led1: Sensors init/error
    // led2: Actuators init/error
    // led3: CanBus init/error
    // led4: Everything ok

    // Start logging when system boots
    std::cout << "Starting Logger" << std::endl;
    if (!sdLogger.start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Logger ***" << std::endl;

        if (!sdLogger.testSDCard())
            std::cerr << "\tReason: SD card not present or not writable"
                      << std::endl;
        else
            std::cerr << "\tReason: Logger initialization error" << std::endl;
    }
    else
    {
        sdLogger.resetStats();
        std::cout << "Logger Ok!\n"
                  << "\tLog number: " << sdLogger.getStats().logNumber
                  << std::endl;
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    std::cout << "Starting Actuators" << std::endl;
    led2On();
    if (!actuators->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Actuators ***" << std::endl;
    }
    else
    {
        led2Off();
    }

    std::cout << "Starting CanHandler" << std::endl;
    led3On();
    if (!canHandler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start CanHandler ***" << std::endl;
    }
    else
    {
        led3Off();
    }

    if (hil)
    {
        if (!hil->start())
        {
            initResult = false;
            std::cerr << "*** Error failed to start HIL ***" << std::endl;
        }

        // Waiting for start of simulation
        hil->waitStartSimulation();
    }

    std::cout << "Starting Sensors" << std::endl;
    led1On();
    if (!sensors->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Sensors ***" << std::endl;
    }
    else
    {
        std::cout << "\tCalibrating sensors" << std::endl;
        sensors->calibrate();
        led1Off();
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
        std::cerr << "*** Init failure ***" << std::endl;
    }

    std::cout << "Sensor status:" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(24) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

    std::cout << "Battery voltage: " << std::fixed << std::setprecision(2)
              << sensors->getBatteryVoltage().voltage << " V" << std::endl;

    // From here on main thread will do non-critical stuff, set lowest priority
    Thread::setPriority(BoardScheduler::Priority::LOW);

    while (true)
    {
        // Log logger and CPU stats
        sdLogger.log(sdLogger.getStats());
        sdLogger.log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();

        // Toggle LED
        gpios::boardLed::value() ? gpios::boardLed::low()
                                 : gpios::boardLed::high();
        Thread::sleep(1000);
    }

    return 0;
}
