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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Buses.h>
#include <Parafoil/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/ADAController/ADAController.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace Common;

int main()
{
    ledOff();

    bool initResult = true;

    DependencyManager manager;

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    Sensors* sensors;
    auto flightModeManager = new FlightModeManager();
    auto nasController     = new NASController();
    auto pinHandler        = new PinHandler();
    auto radio             = new Radio();
    auto landingFlare      = new LandingFlare();
    auto altitudeTrigger   = new AltitudeTrigger();
    auto wingController    = new WingController();
    auto actuators         = new Actuators();
    auto statsRecorder     = new FlightStatsRecorder();
    auto adaController     = new ADAController();

    sensors = new Sensors();

    auto& sdLogger = Logger::getInstance();
    auto& broker   = EventBroker::getInstance();

    // Setup event sniffer
    EventSniffer sniffer(EventBroker::getInstance(), TOPICS_LIST,
                         [&](uint8_t event, uint8_t topic)
                         {
                             EventData data{TimestampTimer::getTimestamp(),
                                            event, topic};
                             sdLogger.log(data);
                         });

    // Insert modules
    initResult &= manager.insert<Buses>(buses) &&
                  manager.insert<BoardScheduler>(scheduler) &&
                  manager.insert<Sensors>(sensors) &&
                  manager.insert<FlightModeManager>(flightModeManager) &&
                  manager.insert<NASController>(nasController) &&
                  manager.insert<ADAController>(adaController) &&
                  manager.insert<PinHandler>(pinHandler) &&
                  manager.insert<Radio>(radio) &&
                  manager.insert<LandingFlare>(landingFlare) &&
                  manager.insert<AltitudeTrigger>(altitudeTrigger) &&
                  manager.insert<WingController>(wingController) &&
                  manager.insert<Actuators>(actuators) &&
                  manager.insert<FlightStatsRecorder>(statsRecorder) &&
                  manager.inject();

    if (!initResult)
    {
        std::cerr << "*** Failed to inject dependencies ***" << std::endl;
        return -1;
    }

    /* Status led indicators
    led1: Sensors init/error
    led2: Radio init/error
    led3: nothing
    led4: Everything ok */

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
        std::cout << "Logger Ok!\n"
                  << "\tLog number: " << sdLogger.getCurrentLogNumber()
                  << std::endl;
    }

    std::cout << "Starting EventBroker" << std::endl;
    if (!broker.start())
    {
        initResult = false;
        std::cerr << "*** Failed to start EventBroker ***" << std::endl;
    }

    std::cout << "Starting PinHandler" << std::endl;
    if (!pinHandler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start PinHandler ***" << std::endl;
    }

    std::cout << "Starting Radio" << std::endl;
    led2On();
    if (!radio->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Radio ***" << std::endl;
    }
    else
    {
        led2Off();
    }

    std::cout << "Starting FlightModeManager" << std::endl;
    if (!flightModeManager->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start FlightModeManager ***" << std::endl;
    }

    std::cout << "Starting NasController" << std::endl;
    if (!nasController->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start NasController ***" << std::endl;
    }

    std::cout << "Starting ADAController" << std::endl;
    if (!adaController->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start ADAController ***" << std::endl;
    }

    std::cout << "Starting AltitudeTrigger" << std::endl;
    if (!altitudeTrigger->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start AltitudeTrigger ***" << std::endl;
    }

    std::cout << "Starting LandingFlare" << std::endl;
    if (!landingFlare->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start LandingFlare ***" << std::endl;
    }

    std::cout << "Starting WingController" << std::endl;
    if (!wingController->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start WingController ***" << std::endl;
    }

    std::cout << "Starting Actuators" << std::endl;
    if (!actuators->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Actuators ***" << std::endl;
    }

    std::cout << "Starting Scheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Scheduler ***" << std::endl;
    }

    // Wait for simulation start before starting sensors to avoid initializing
    // them with invalid data
    std::cout << "Starting Sensors" << std::endl;
    led1On();
    if (!sensors->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Sensors ***" << std::endl;
    }
    else
    {
        led1Off();
    }

    if (initResult)
    {
        broker.post(FMM_INIT_OK, TOPIC_FMM);
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        broker.post(FMM_INIT_ERROR, TOPIC_FMM);
        std::cerr << "*** Init failure ***" << std::endl;
    }

    auto sensorConfig = Config::Sensors::USING_DUAL_MAGNETOMETER
                            ? "LIS2MDL IN + LIS2MDL EXT"
                            : "LPS22DF + LIS2MDL IN/EXT";

    std::cout << "Sensor status (config: " << sensorConfig << "):" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(20) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

    std::cout << "Battery voltage: " << std::fixed << std::setprecision(2)
              << sensors->getBatteryVoltageLastSample().voltage << " V"
              << std::endl;

    // Collect stack usage statistics
    while (true)
    {
        sdLogger.log(sdLogger.getStats());

        // Toggle LED
        gpios::boardLed::value() ? gpios::boardLed::low()
                                 : gpios::boardLed::high();
        Thread::sleep(1000);
    }

    return 0;
}
