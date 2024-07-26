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
#include <Payload/VerticalVelocityTrigger/VerticalVelocityTrigger.h>
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

#include <iostream>

#if defined(DEBUG)
// Use the logger to print initialization messages in debug mode

// Main logging
#define LOG_ACTION(msg) LOG_INFO(logger, msg)
#define LOG_RESULT(success) ((void)0)

// Module start logging
#define LOG_START(msg) LOG_INFO(logger, msg)
#define LOG_START_RESULT(success, ...) \
    if (!success)                      \
    {                                  \
        LOG_ERR(logger, __VA_ARGS__);  \
    }

// Final status logging
#define LOG_SUCCESS(msg) LOG_INFO(logger, msg)
#define LOG_FAILURE(msg) LOG_ERR(logger, msg)

#else
// Print initialization messages directly in release mode

// Main logging
#define LOG_ACTION(msg) std::cout << msg << "... " << std::flush
#define LOG_RESULT(success) \
    std::cout << (success ? "Ok" : "Failed") << std::endl

// Module start logging
#define LOG_START(msg) std::cout << msg << "... " << std::flush
#define LOG_START_RESULT(success, ...) \
    std::cout << (success ? "Ok" : "Failed") << std::endl

// Final status logging
#define LOG_SUCCESS(msg) std::cout << msg << std::endl
#define LOG_FAILURE(msg) std::cout << msg << std::endl

#endif

/**
 * @brief Starts a module and checks if it started correctly.
 * Must be followed by a semicolon or a block of code.
 * The block of code will be executed only if the module started correctly.
 * The macro also defines a boolean variable of the same name as the module
 * followed by "Started" with the result of the start operation, e.g.
 * `sensorsStarted`.
 *
 * @example START_MODULE(sensors) { miosix::ledOn(); }
 */
#define START_MODULE(module)                                       \
    LOG_START("Starting " #module);                                \
    bool module##Started = module->start();                        \
    initResult &= module##Started;                                 \
    LOG_START_RESULT(module##Started, "Failed to start " #module); \
    if (module##Started)

/**
 * @brief Starts a singleton and checks if it started correctly.
 * Must be followed by a semicolon or a block of code.
 * The block of code will be executed only if the singleton started correctly.
 * The macro also defines a boolean variable of the same name as the singleton
 * followed by "Started" with the result of the start operation, e.g.
 * `LoggerStarted`.
 *
 * @example `START_SINGLETON(Logger) { miosix::ledOn(); }`
 */
#define START_SINGLETON(singleton)                                       \
    LOG_START("Starting " #singleton);                                   \
    bool singleton##Started = singleton::getInstance().start();          \
    initResult &= singleton##Started;                                    \
    LOG_START_RESULT(singleton##Started, "Failed to start " #singleton); \
    if (singleton##Started)

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

    LOG_ACTION("Instantiating modules");
    // Core components
    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    // Global state machine
    auto flightModeManager = new FlightModeManager();

    // Attitude estimation
    auto nas = new NASController();

    // Sensors
    auto sensors    = new Sensors();
    auto pinHandler = new PinHandler();

    // Radio and CAN
    auto radio      = new Radio();
    auto canHandler = new CanHandler();

    // Flight algorithms
    auto altitudeTrigger         = new AltitudeTrigger();
    auto wingController          = new WingController();
    auto verticalVelocityTrigger = new VerticalVelocityTrigger();
    auto windEstimation          = new WindEstimation();

    // Actuators
    auto actuators = new Actuators();

    // Statistics
    auto statsRecorder = new FlightStatsRecorder();

    LOG_RESULT(true);

    LOG_ACTION("Injecting module dependencies");
    // Insert modules
    bool initResult = depman.insert(buses) && depman.insert(scheduler) &&
                      depman.insert(flightModeManager) && depman.insert(nas) &&
                      depman.insert(sensors) && depman.insert(pinHandler) &&
                      depman.insert(radio) && depman.insert(canHandler) &&
                      depman.insert(altitudeTrigger) &&
                      depman.insert(wingController) &&
                      depman.insert(verticalVelocityTrigger) &&
                      depman.insert(windEstimation) &&
                      depman.insert(actuators) && depman.insert(statsRecorder);

    // Populate module dependencies
    initResult &= depman.inject();
    LOG_RESULT(initResult);

    /* Status led indicators
    led1: Sensors ok
    led2: Radio ok
    led3: CanBus ok
    led4: Everything ok */

    // Check SD card presence for logging
    LOG_START("Testing Logger SD card");
    bool sdCardOk = Logger::getInstance().testSDCard();
    initResult &= sdCardOk;
    LOG_START_RESULT(sdCardOk, "No SD card detected, Logger will not work");

    // Start global modules
    START_SINGLETON(EventBroker);
    // Start module instances
    START_MODULE(sensors) { miosix::led1On(); }
    START_MODULE(pinHandler);
    START_MODULE(radio) { miosix::led2On(); }
    START_MODULE(canHandler) { miosix::led3On(); }
    START_MODULE(flightModeManager);
    START_MODULE(nas);
    START_MODULE(altitudeTrigger);
    START_MODULE(wingController);
    START_MODULE(verticalVelocityTrigger);
    START_MODULE(windEstimation);
    START_MODULE(actuators);
    START_MODULE(statsRecorder);

    START_MODULE(scheduler);

    // Log all posted events
    LOG_ACTION("Starting event sniffer");
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });
    LOG_RESULT(true);

    if (initResult)
    {
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
        // Turn on the initialization led on the CU
        miosix::led4On();
        actuators->setStatusOk();
        LOG_SUCCESS("Initialization successful");
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        actuators->setStatusError();
        LOG_FAILURE("Initialization failed");
    }

    auto toggleBoardLed = [on = false]() mutable
    {
        if ((on = !on))
            miosix::gpios::boardLed::low();
        else
            miosix::gpios::boardLed::high();
    };

    // Collect CPU and stack usage statistics
    while (true)
    {
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        StackLogger::getInstance().log();
        toggleBoardLed();
        Thread::sleep(1000);
    }

    return 0;
}
