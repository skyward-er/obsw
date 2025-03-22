/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <MockupMain/BoardScheduler.h>
#include <MockupMain/Buses.h>
#include <MockupMain/FlightStatsRecorder/FlightStatsRecorder.h>
#include <MockupMain/PinHandler/PinHandler.h>
#include <MockupMain/Radio/Radio.h>
#include <MockupMain/Sensors/Sensors.h>
#include <MockupMain/StateMachines/FlightModeManager/FlightModeManager.h>
#include <MockupMain/StateMachines/NASController/NASController.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <events/EventBroker.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <iomanip>
#include <iostream>

/**
 * @brief Starts a module and checks if it started correctly.
 * Must be followed by a semicolon or a block of code.
 * The block of code will be executed only if the module started correctly.
 *
 * @example START_MODULE(sensors) { miosix::ledOn(); }
 */
#define START_MODULE(module)                                             \
    std::cout << "Starting " #module << std::endl;                       \
    if (!module->start())                                                \
    {                                                                    \
        initResult = false;                                              \
        std::cerr << "*** Failed to start " #module " ***" << std::endl; \
    }                                                                    \
    else

/**
 * @brief Starts a singleton and checks if it started correctly.
 * Must be followed by a semicolon or a block of code.
 * The block of code will be executed only if the singleton started correctly.
 *
 * @example `START_SINGLETON(Logger) { miosix::ledOn(); }`
 */
#define START_SINGLETON(singleton)                                          \
    std::cout << "Starting " #singleton << std::endl;                       \
    if (!singleton::getInstance().start())                                  \
    {                                                                       \
        initResult = false;                                                 \
        std::cerr << "*** Failed to start " #singleton " ***" << std::endl; \
    }

// Build type string for printing during startup
#if defined(DEBUG)
#define BUILD_TYPE "Debug"
#else
#define BUILD_TYPE "Release"
#endif

using namespace Boardcore;
using namespace MockupMain;
using namespace Common;

int main()
{
    miosix::ledOff();
    std::cout << "MockupMain Entrypoint " << "(" << BUILD_TYPE << ")"
              << " by Skyward Experimental Rocketry" << std::endl;

    // cppcheck-suppress unreadVariable
    auto logger = Logging::getLogger("Parafoil");
    DependencyManager depman{};

    std::cout << "Instantiating modules" << std::endl;
    bool initResult = true;

    // Core components
    auto buses = new Buses();
    initResult &= depman.insert(buses);
    auto scheduler = new BoardScheduler();
    initResult &= depman.insert(scheduler);

    // Global state machine
    auto flightModeManager = new FlightModeManager();
    initResult &= depman.insert(flightModeManager);

    // Attitude estimation
    auto nasController = new NASController();
    initResult &= depman.insert(nasController);

    // Sensors
    auto sensors = new Sensors();
    initResult &= depman.insert(sensors);
    auto pinHandler = new PinHandler();
    initResult &= depman.insert(pinHandler);

    // Radio
    auto radio = new Radio();
    initResult &= depman.insert(radio);

    // Stats recorder
    auto flightStatsRecorder = new FlightStatsRecorder();
    initResult &= depman.insert(flightStatsRecorder);

    std::cout << "Injecting module dependencies" << std::endl;
    initResult &= depman.inject();

    START_SINGLETON(Logger)
    {
        std::cout << "Logger Ok!\n"
                  << "\tLog number: "
                  << Logger::getInstance().getCurrentLogNumber() << std::endl;
    }
    START_SINGLETON(EventBroker);

    START_MODULE(pinHandler);
    START_MODULE(radio);
    START_MODULE(nasController);
    START_MODULE(flightModeManager);

    START_MODULE(scheduler);

    START_MODULE(sensors);

    if (initResult)
    {
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
        // Turn on the initialization led on the CU
        miosix::ledOn();
        std::cout << "Parafoil initialization Ok!" << std::endl;
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        std::cerr << "*** Parafoil initialization error ***" << std::endl;
    }

    std::cout << "Sensors status:" << std::endl;
    auto sensorInfo = sensors->getSensorInfo();
    for (const auto& info : sensorInfo)
    {
        std::cout << "\t" << std::setw(16) << std::left << info.id << " "
                  << (info.isInitialized ? "Ok" : "Error") << "\n";
    }
    std::cout.flush();

    // Collect stack usage statistics
    while (true)
    {
        StackLogger::getInstance().log();
        Thread::sleep(1000);
    }

    return 0;
}
