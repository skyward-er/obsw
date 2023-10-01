/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <RIG/Actuators/Actuators.h>
#include <RIG/BoardScheduler.h>
#include <RIG/Buses.h>
#include <RIG/CanHandler/CanHandler.h>
#include <RIG/Radio/Radio.h>
#include <RIG/Sensors/Sensors.h>
#include <RIG/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIG/StateMachines/TARS1/TARS1.h>
#include <RIG/StatesMonitor/StatesMonitor.h>
#include <RIG/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <logger/Logger.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace RIG;
using namespace miosix;
using namespace Common;

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status. If at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");

    // Create modules
    Buses* buses              = new Buses();
    BoardScheduler* scheduler = new BoardScheduler();
    Actuators* actuators      = new Actuators(scheduler->getScheduler(4));
    Sensors* sensors          = new Sensors(scheduler->getScheduler(3));
    Radio* radio              = new Radio();
    TMRepository* tmRepo      = new TMRepository();
    GroundModeManager* groundModeManager = new GroundModeManager();
    TARS1* tars = new TARS1(scheduler->getScheduler(4));
    CanHandler* can =
        new CanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));
    StatesMonitor* states =
        new StatesMonitor(scheduler->getScheduler(miosix::MAIN_PRIORITY));

    // Initialize singleton modules
    Logger& SDlogger    = Logger::getInstance();
    EventBroker& broker = EventBroker::getInstance();

    // Insert modules
    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the buses module");
    }

    if (!modules.insert<BoardScheduler>(scheduler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the scheduler module");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the sensors module");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the actuators module");
    }

    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the radio module");
    }

    if (!modules.insert<TMRepository>(tmRepo))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the tm repository module");
    }

    if (!modules.insert<GroundModeManager>(groundModeManager))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the ground mode manager FSM module");
    }

    if (!modules.insert<TARS1>(tars))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the tars module");
    }

    if (!modules.insert<CanHandler>(can))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the CAN module");
    }

    if (!modules.insert<StatesMonitor>(states))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the States Monitor module");
    }

    // Start singleton modules
    if (!SDlogger.testSDCard())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the SD logger");
    }

    if (!broker.start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Event Broker");
    }

    // Start modules
    if (!modules.get<Actuators>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the actuators module");
    }

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the board scheduler");
    }

    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors module");
    }

    if (!modules.get<StatesMonitor>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the States Monitor module");
    }

    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio module");
    }

    if (!modules.get<GroundModeManager>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the ground mode manager module");
    }

    if (!modules.get<TARS1>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the tars module");
    }

    if (!modules.get<CanHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CAN module");
    }

    if (initResult)
    {
        // POST OK
        ui::redLed::low();
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_MOTOR);
        LOG_INFO(logger, "Init success!");
    }
    else
    {
        // POST ERROR
        ui::redLed::high();
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_MOTOR);
    }

    // Log all events
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        StackLogger::getInstance().log();
    }

    return 0;
}