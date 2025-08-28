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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Buses.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Radio/Radio.h>
#include <RIGv2/Registry/Registry.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIGv2/StateMachines/TARS1/TARS1.h>
#include <RIGv2/StateMachines/TARS3/TARS3.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;
using namespace RIGv2;
using namespace miosix;

int main()
{
    ledOff();

    bool initResult = true;

    DependencyManager manager;

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    auto sensors     = new Sensors();
    auto actuators   = new Actuators();
    auto registry    = new Registry();
    auto canHandler  = new CanHandler();
    auto gmm         = new GroundModeManager();
    auto tars1       = new TARS1();
    auto tars3       = new TARS3();
    auto radio       = new Radio();
    auto motorStatus = new MotorStatus();

    auto& sdLogger = Logger::getInstance();
    auto& broker   = EventBroker::getInstance();

    // Setup event sniffer
    EventSniffer sniffer(broker,
                         [&](uint8_t event, uint8_t topic)
                         {
                             EventData data{TimestampTimer::getTimestamp(),
                                            event, topic};
                             sdLogger.log(data);
                         });

    // Insert modules
    initResult &=
        manager.insert<Buses>(buses) &&
        manager.insert<BoardScheduler>(scheduler) &&
        manager.insert<Actuators>(actuators) &&
        manager.insert<Sensors>(sensors) && manager.insert<Radio>(radio) &&
        manager.insert<CanHandler>(canHandler) &&
        manager.insert<Registry>(registry) &&
        manager.insert<GroundModeManager>(gmm) &&
        manager.insert<TARS1>(tars1) && manager.insert<TARS3>(tars3) &&
        manager.insert<MotorStatus>(motorStatus) && manager.inject();

    if (!initResult)
    {
        std::cerr << "*** Failed to inject dependencies ***" << std::endl;
        return -1;
    }

    // Status led indicators
    // led1: Sensors init/error
    // led2: Radio init/error
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

    // Start modules
    std::cout << "Starting EventBroker" << std::endl;
    if (!broker.start())
    {
        initResult = false;
        std::cerr << "*** Failed to start EventBroker ***" << std::endl;
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    std::cout << "Starting Registry" << std::endl;
    if (!registry->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Registry ***" << std::endl;
    }

    // Perform an initial registry load
    std::cout << "Loading backed registry" << std::endl;
    if (registry->load() != RegistryError::OK)
        std::cout << "* Warning: could not load a saved registry *"
                  << std::endl;

    std::cout << "Starting Actuators" << std::endl;
    if (!actuators->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Actuators ***" << std::endl;
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

    std::cout << "Starting GroundModeManager" << std::endl;
    if (!gmm->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start GroundModeManager ***" << std::endl;
    }

    std::cout << "Starting TARS1" << std::endl;
    if (!tars1->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start TARS1 ***" << std::endl;
    }

    std::cout << "Starting TARS3" << std::endl;
    if (!tars3->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start TARS3 ***" << std::endl;
    }

    if (initResult)
    {
        broker.post(FMM_INIT_OK, TOPIC_MOTOR);
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        broker.post(FMM_INIT_ERROR, TOPIC_MOTOR);
        std::cerr << "*** Init failure ***" << std::endl;
    }

    std::cout << "Sensor status:" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(20) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        sdLogger.log(sdLogger.getStats());
        sdLogger.log(radio->getMavStatus());
    }

    return 0;
}
