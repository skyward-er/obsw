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

#include <Payload/Actuators/Actuators.h>
#include <Payload/AltitudeTrigger/AltitudeTrigger.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Buses.h>
#include <Payload/CanHandler/CanHandler.h>
#include <Payload/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Payload/HIL/HIL.h>
#include <Payload/PersistentVars/PersistentVars.h>
#include <Payload/PinHandler/PinHandler.h>
#include <Payload/Radio/Radio.h>
#include <Payload/Sensors/HILSensors.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/StateMachines/ZVKController/ZVKController.h>
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
using namespace Payload;
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
    auto zvkController     = new ZVKController();
    auto pinHandler        = new PinHandler();
    auto radio             = new Radio();
    auto canHandler        = new CanHandler();
    auto landingFlare      = new LandingFlare();
    auto altitudeTrigger   = new AltitudeTrigger();
    auto wingController    = new WingController();
    auto actuators         = new Actuators();
    auto statsRecorder     = new FlightStatsRecorder();
    PayloadHIL* hil        = nullptr;

    // HIL
    if (PersistentVars::getHilMode())
    {
        std::cout << "PAYLOAD SimulatorData: " << sizeof(SimulatorData)
                  << ", ActuatorData: " << sizeof(ActuatorData) << std::endl;

        hil = new PayloadHIL();
        initResult &= manager.insert<PayloadHIL>(hil);
        sensors = new HILSensors(Config::HIL::ENABLE_HW);
    }
    else
    {
        sensors = new Sensors();
    }

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
                  manager.insert<ZVKController>(zvkController) &&
                  manager.insert<PinHandler>(pinHandler) &&
                  manager.insert<Radio>(radio) &&
                  manager.insert<CanHandler>(canHandler) &&
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
    led3: CanBus init/error
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

    std::cout << "Starting ZvkController" << std::endl;
    if (!zvkController->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start ZVkController ***" << std::endl;
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

    if (hil)
    {
        std::cout << "Starting HIL" << std::endl;
        if (!hil->start())
        {
            initResult = false;
            std::cerr << "*** Failed to start HIL ***" << std::endl;
        }

        std::cout << "Waiting simulation start..." << std::endl;
        hil->waitStartSimulation();
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
