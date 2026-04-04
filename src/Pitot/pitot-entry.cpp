/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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

#include <Pitot/BoardScheduler.h>
#include <Pitot/Buses.h>
#include <Pitot/CanHandler/CanHandler.h>
#include <Pitot/Sensors/Sensors.h>
#include <common/Topics.h>
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
using namespace Pitot;
using namespace Common;

int main()
{
    ledOff();

    bool initResult = true;

    DependencyManager manager;

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();

    Sensors* sensors;
    auto canHandler = new CanHandler();

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
                  manager.insert<CanHandler>(canHandler) && manager.inject();

    if (!initResult)
    {
        std::cerr << "*** Failed to inject dependencies ***" << std::endl;
        return -1;
    }

    /* Status led indicators
    led1: Sensors init/error
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

    std::cout << "Starting Scheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Scheduler ***" << std::endl;
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

    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(20) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

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
