/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <MockupMain/PinHandler/PinHandler.h>
#include <MockupMain/Radio/Radio.h>
#include <MockupMain/Sensors/Sensors.h>
#include <MockupMain/StateMachines/FlightModeManager/FlightModeManager.h>
#include <MockupMain/StateMachines/NASController/NASController.h>
#include <MockupMain/TMRepository/TMRepository.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace MockupMain;
using namespace Common;

int main()
{
    Thread::sleep(500);  // wait 500 ms to separate the mcu startup and the ubx
                         // serial communication

    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("Parafoil");

    // Scheduler
    BoardScheduler* scheduler = new BoardScheduler();

    // Nas priority (Max priority)
    NASController* nas =
        new NASController(scheduler->getScheduler(miosix::PRIORITY_MAX));

    // Sensors priority (MAX - 1)
    Sensors* sensors =
        new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));

    // Other critical components (Max - 2)
    Radio* radio = new Radio(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));

    // Components without a scheduler
    TMRepository* tmRepo   = new TMRepository();
    FlightModeManager* fmm = new FlightModeManager();
    Buses* buses           = new Buses();
    PinHandler* pinHandler = new PinHandler();

    // Insert modules
    if (!modules.insert<BoardScheduler>(scheduler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Board Scheduler module");
    }

    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Buses module");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Sensor module");
    }

    if (!modules.insert<NASController>(nas))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the NAS module");
    }

    if (!modules.insert<FlightModeManager>(fmm))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the FMM module");
    }
    if (!modules.insert<Radio>(radio))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the Radio module");
    }

    if (!modules.insert<TMRepository>(tmRepo))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the TMRepository module");
    }

    if (!modules.insert<PinHandler>(pinHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the PinHandler module");
    }

    // Start modules
    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Logger module");
    }
    else
    {
        LOG_DEBUG(logger, "Logger started");
    }

    if (!EventBroker::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the EventBroker module");
    }

    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Sensors module");
    }

    // if (!modules.get<NASController>()->start())
    // {
    //     initResult = false;
    //     LOG_ERR(logger, "Error starting the NAS module");
    // }

    if (!modules.get<FlightModeManager>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the FMM module");
    }

    if (!modules.get<Radio>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Radio module");
    }

    // if (!modules.get<PinHandler>()->start())
    // {
    //     initResult = false;
    //     LOG_ERR(logger, "Error starting the PinHandler module");
    // }

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the Board Scheduler module");
    }

    // // Log all the events
    // EventSniffer sniffer(EventBroker::getInstance(), TOPICS_LIST,
    //                      [](uint8_t event, uint8_t topic)
    //                      {
    //                          EventData ev{TimestampTimer::getTimestamp(), event,
    //                                       topic};
    //                          Logger::getInstance().log(ev);
    //                      });

    // Check the init result and launch an event
    if (initResult)
    {
        // Post OK
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
    }
    else
    {
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);
        LOG_ERR(logger, "Failed to initialize");
    }

    // Periodic statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        modules.get<Radio>()->logStatus();
    }

    return 0;
}
