/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#define DEFAULT_STDOUT_LOG_LEVEL LOGL_INFO

#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
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
    }                                                                       \
    else

// Build type string for printing during startup
#if defined(DEBUG)
#define BUILD_TYPE "Debug"
#else
#define BUILD_TYPE "Release"
#endif

// Build flavor string for printing during startup
#if defined(EUROC)
#define FLAVOR "EUROC"
#elif defined(ROCCARASO)
#define FLAVOR "ROCCARASO"
#else
#define FLAVOR "MILAN"
#endif

using namespace Boardcore;
using namespace Payload;
using namespace Common;

int main()
{
    miosix::ledOff();
    std::cout << "Payload " << FLAVOR << " Entrypoint "
              << "(" << BUILD_TYPE << ")"
              << " by Skyward Experimental Rocketry" << std::endl;

    // Unused but needed to set the log level properly
    auto logger = Logging::getLogger("Payload");
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

    // Radio and CAN
    auto radio = new Radio();
    initResult &= depman.insert(radio);
    auto canHandler = new CanHandler();
    initResult &= depman.insert(canHandler);

    // Flight algorithms
    auto altitudeTrigger = new AltitudeTrigger();
    initResult &= depman.insert(altitudeTrigger);
    auto wingController = new WingController();
    initResult &= depman.insert(wingController);
    auto windEstimation = new WindEstimation();
    initResult &= depman.insert(windEstimation);

    // Actuators
    auto actuators = new Actuators();
    initResult &= depman.insert(actuators);

    // Statistics
    auto statsRecorder = new FlightStatsRecorder();
    initResult &= depman.insert(statsRecorder);

    std::cout << "Injecting module dependencies" << std::endl;
    initResult &= depman.inject();

    /* Status led indicators
    led1: Sensors ok
    led2: Radio ok
    led3: CanBus ok
    led4: Everything ok */

    // Start global modules
    START_SINGLETON(Logger)
    {
        std::cout << "Logger Ok!\n"
                  << "\tLog number: "
                  << Logger::getInstance().getCurrentLogNumber() << std::endl;
    }
    START_SINGLETON(EventBroker);

    // Start module instances
    START_MODULE(sensors) { miosix::led1On(); }
    START_MODULE(pinHandler);
    START_MODULE(radio) { miosix::led2On(); }
    START_MODULE(canHandler) { miosix::led3On(); }
    START_MODULE(flightModeManager);
    START_MODULE(nasController);
    START_MODULE(altitudeTrigger);
    START_MODULE(wingController);
    START_MODULE(windEstimation);
    START_MODULE(actuators);

    START_MODULE(scheduler);

    // Log all posted events
    std::cout << "Starting event sniffer" << std::endl;
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    if (initResult)
    {
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
        // Turn on the initialization led on the CU
        miosix::led4On();
        actuators->setStatusOk();
        std::cout << "Payload initialization Ok!" << std::endl;
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        actuators->setStatusError();
        std::cerr << "*** Payload initialization error ***" << std::endl;
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
